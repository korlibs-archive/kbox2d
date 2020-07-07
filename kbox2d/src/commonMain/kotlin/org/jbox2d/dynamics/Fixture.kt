/*******************************************************************************
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package org.jbox2d.dynamics

import org.jbox2d.collision.*
import org.jbox2d.collision.broadphase.*
import org.jbox2d.collision.shapes.*
import org.jbox2d.common.*
import org.jbox2d.internal.*
import org.jbox2d.userdata.*

/**
 * A fixture is used to attach a shape to a body for collision detection. A fixture inherits its
 * transform from its parent. Fixtures hold additional non-geometric data such as friction,
 * collision filters, etc. Fixtures are created via Body::createFixture.
 *
 * @warning you cannot reuse fixtures.
 *
 * @author daniel
 */
class Fixture : Box2dTypedUserData by Box2dTypedUserData.Mixin() {

    var _density: Float = 0f

    /**
     * Next fixture in the parent body's fixture list.
     */
    var next: Fixture? = null

    /**
     * Parent body of this fixture. This is null if the fixture is not attached.
     */
    var body: Body? = null

    /**
     * Child shape. You can modify the child shape, however you should not change the number
     * of vertices because this will crash some collision caching mechanisms.
     */
    var shape: Shape? = null

    /**
     * Coefficient of friction. Changing this value will _not_ change the friction of existing contacts.
     */
    var friction: Float = 0f

    /**
     * Coefficient of restitution. Changing this value will _not_ change the restitution of existing
     * contacts.
     */
    var restitution: Float = 0f

    var proxies: Array<FixtureProxy>? = null

    var proxyCount: Int = 0

    val filter: Filter = Filter()

    var _isSensor: Boolean = false

    /**
     * User data that was assigned in the fixture definition. Use this to store your
     * application specific data.
     */
    var userData: Any? = null

    /**
     * Type of the child shape. You can use this to down cast to the concrete shape.
     */
    val type: ShapeType
        get() = shape!!.type

    /**
     * Whether this fixture is a sensor (non-solid)
     */
    var isSensor: Boolean
        get() = _isSensor
        set(sensor) {
            if (sensor != _isSensor) {
                body!!.isAwake = true
                _isSensor = sensor
            }
        }

    /**
     * Contact filtering data. Changing this value is an expensive operation and should not be called
     * frequently. This will not update contacts until the next time step when either parent body is
     * awake. This automatically calls [refilter].
     */
    var filterData: Filter
        get() = filter
        set(filter) {
            this.filter.set(filter)
            refilter()
        }

    var density: Float
        get() = _density
        set(density) {
            assert(density >= 0f)
            _density = density
        }

    private val pool1 = AABB()
    private val pool2 = AABB()
    private val displacement = Vec2()

    /**
     * Call this if you want to establish collision that was previously disabled by
     * ContactFilter::ShouldCollide.
     */
    fun refilter() {
        if (body == null) return

        // Flag associated contacts for filtering.
        var edge = body!!.contactList
        while (edge != null) {
            val contact = edge.contact
            val fixtureA = contact!!.fixtureA
            val fixtureB = contact.fixtureB
            if (fixtureA === this || fixtureB === this) {
                contact.flagForFiltering()
            }
            edge = edge.next
        }

        val world = body!!.world

        // Touch each proxy so that new pairs may be created
        val broadPhase = world.contactManager.broadPhase
        for (i in 0 until proxyCount) {
            broadPhase.touchProxy(proxies!![i].proxyId)
        }
    }

    /**
     * Test a point (in world coordinates) for containment in this fixture.
     * This only works for convex shapes.
     */
    fun testPoint(p: Vec2): Boolean {
        return shape!!.testPoint(body!!.xf, p)
    }

    /**
     * Cast a ray against this shape.
     *
     * @param output the ray-cast results.
     * @param input the ray-cast input parameters.
     */
    fun raycast(output: RayCastOutput, input: RayCastInput, childIndex: Int): Boolean {
        return shape!!.raycast(output, input, body!!.xf, childIndex)
    }

    /**
     * Get the mass data for this fixture. The mass data is based on the density and the shape. The
     * rotational inertia is about the shape's origin.
     */
    fun getMassData(massData: MassData) {
        shape!!.computeMass(massData, _density)
    }

    /**
     * Get the fixture's AABB. This AABB may be enlarge and/or stale. If you need a more accurate
     * AABB, compute it using the shape and the body transform.
     */
    fun getAABB(childIndex: Int): AABB {
        assert(childIndex in 0 until proxyCount)
        return proxies!![childIndex].aabb
    }

    /**
     * Compute the distance from this fixture.
     *
     * @param p a point in world coordinates.
     */
    fun computeDistance(p: Vec2, childIndex: Int, normalOut: Vec2): Float {
        return shape!!.computeDistanceToOut(body!!.getTransform(), p, childIndex, normalOut)
    }

    // We need separation create/destroy functions from the constructor/destructor because
    // the destructor cannot access the allocator (no destructor arguments allowed by C++).

    fun create(body: Body, def: FixtureDef) {
        userData = def.userData
        friction = def.friction
        restitution = def.restitution

        this.body = body
        next = null

        filter.set(def.filter)

        _isSensor = def.isSensor

        shape = def.shape!!.clone()

        // Reserve proxy space
        val childCount = shape!!.getChildCount()
        if (proxies == null) {
            proxies = Array(childCount) { FixtureProxy() }
            for (i in 0 until childCount) {
                proxies!![i].fixture = null
                proxies!![i].proxyId = BroadPhase.NULL_PROXY
            }
        }

        if (proxies!!.size < childCount) {
            val old = proxies
            val newLen = MathUtils.max(old!!.size * 2, childCount)
            proxies = arrayOfNulls<FixtureProxy>(newLen) as Array<FixtureProxy>
            arraycopy(old, 0, proxies!!, 0, old.size)
            for (i in 0 until newLen) {
                if (i >= old.size) {
                    proxies!![i] = FixtureProxy()
                }
                proxies!![i].fixture = null
                proxies!![i].proxyId = BroadPhase.NULL_PROXY
            }
        }
        proxyCount = 0

        _density = def.density
    }

    fun destroy() {
        // The proxies must be destroyed before calling this.
        assert(proxyCount == 0)

        // Free the child shape.
        shape = null
        proxies = null
        next = null

        // TODO pool shapes
        // TODO pool fixtures
    }

    // These support body activation/deactivation.
    fun createProxies(broadPhase: BroadPhase, xf: Transform) {
        assert(proxyCount == 0)

        // Create proxies in the broad-phase.
        proxyCount = shape!!.getChildCount()

        for (i in 0 until proxyCount) {
            val proxy = proxies!![i]
            shape!!.computeAABB(proxy.aabb, xf, i)
            proxy.proxyId = broadPhase.createProxy(proxy.aabb, proxy)
            proxy.fixture = this
            proxy.childIndex = i
        }
    }

    /**
     * Internal method
     */
    fun destroyProxies(broadPhase: BroadPhase) {
        // Destroy proxies in the broad-phase.
        for (i in 0 until proxyCount) {
            val proxy = proxies!![i]
            broadPhase.destroyProxy(proxy.proxyId)
            proxy.proxyId = BroadPhase.NULL_PROXY
        }

        proxyCount = 0
    }

    /**
     * Internal method
     */
    fun synchronize(broadPhase: BroadPhase, transform1: Transform, transform2: Transform) {
        if (proxyCount == 0) return

        for (i in 0 until proxyCount) {
            val proxy = proxies!![i]

            // Compute an AABB that covers the swept shape (may miss some rotation effect).
            val aabb1 = pool1
            val aab = pool2
            shape!!.computeAABB(aabb1, transform1, proxy.childIndex)
            shape!!.computeAABB(aab, transform2, proxy.childIndex)

            proxy.aabb.lowerBound.x = if (aabb1.lowerBound.x < aab.lowerBound.x) aabb1.lowerBound.x else aab.lowerBound.x
            proxy.aabb.lowerBound.y = if (aabb1.lowerBound.y < aab.lowerBound.y) aabb1.lowerBound.y else aab.lowerBound.y
            proxy.aabb.upperBound.x = if (aabb1.upperBound.x > aab.upperBound.x) aabb1.upperBound.x else aab.upperBound.x
            proxy.aabb.upperBound.y = if (aabb1.upperBound.y > aab.upperBound.y) aabb1.upperBound.y else aab.upperBound.y
            displacement.x = transform2.p.x - transform1.p.x
            displacement.y = transform2.p.y - transform1.p.y

            broadPhase.moveProxy(proxy.proxyId, proxy.aabb, displacement)
        }
    }
}
