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

import com.soywiz.korma.geom.*
import org.jbox2d.collision.shapes.MassData
import org.jbox2d.collision.shapes.Shape
import org.jbox2d.common.MathUtils
import org.jbox2d.common.Rot
import org.jbox2d.common.Sweep
import org.jbox2d.common.Transform
import org.jbox2d.common.Vec2
import org.jbox2d.dynamics.contacts.ContactEdge
import org.jbox2d.dynamics.joints.JointEdge
import org.jbox2d.internal.*
import org.jbox2d.userdata.*

/**
 * A rigid body. These are created via World.createBody.
 *
 * @author Daniel Murphy
 */
class Body(bd: BodyDef, var world: World) : Box2dTypedUserData by Box2dTypedUserData.Mixin() {

    var _type: BodyType

    var flags: Int = 0

    var islandIndex: Int = 0

    /**
     * The body origin transform.
     */
    val xf = Transform()

    fun getTransform() = xf

    /**
     * The previous transform for particle simulation
     */
    val xf0 = Transform()

    /**
     * The swept motion for CCD
     */
    val sweep = Sweep()

    val _linearVelocity = Vec2()
    var _angularVelocity = 0f

    val force = Vec2()
    var torque = 0f

    var prev: Body? = null
    var next: Body? = null

    /** List of all fixtures attached to this body.  */
    var fixtureList: Fixture? = null

    var fixtureCount: Int = 0

    /** List of all joints attached to this body.  */
    var jointList: JointEdge? = null

    /**
     * List of all contacts attached to this body.
     *
     * @warning this list changes during the time step and you may miss some collisions if you don't
     * use ContactListener.
     */
    var contactList: ContactEdge? = null

    /**
     * Total mass of the body, usually in kilograms (kg).
     */
    var mass: Float = 0f
    var invMass: Float = 0f

    /** Rotational inertia about the center of mass. */
    var I: Float = 0f
    var invI: Float = 0f

    /** Linear damping of the body. */
    var linearDamping: Float = 0f

    /** Angular damping of the body. */
    var angularDamping: Float = 0f

    /**
     * Gravity scale of the body.
     */
    var gravityScale: Float = 0f

    var sleepTime: Float = 0f

    /**
     * User data pointer that was provided in the body definition.
     * Use this to store your application specific data.
     */
    var userData: Any? = null

    private val fixDef = FixtureDef()

    /**
     * World body origin position
     */
    val position: Vec2
        get() = xf.p

    /**
     * Current world rotation angle in radians.
     */
    val angleRadians: Float
        get() = sweep.a

    /**
     * Current world rotation angle in degrees.
     */
    val angleDegrees: Float get() = angleRadians * MathUtils.RAD2DEG

    val angle: Angle get() = angleRadians.radians

    /**
     * World position of the center of mass.
     */
    val worldCenter: Vec2
        get() = sweep.c

    /**
     * Local position of the center of mass.
     */
    val localCenter: Vec2
        get() = sweep.localCenter

    /**
     * Linear velocity of the center of mass.
     */
    var linearVelocity: Vec2
        get() = _linearVelocity
        set(v) {
            if (_type === BodyType.STATIC) return

            if (Vec2.dot(v, v) > 0.0f) {
                isAwake = true
            }
            _linearVelocity.set(v)
        }

    /**
     * Angular velocity in radians/second.
     */
    var angularVelocity: Float
        get() = _angularVelocity
        set(w) {
            if (_type === BodyType.STATIC) return

            if (w * w > 0f) {
                isAwake = true
            }
            _angularVelocity = w
        }

    /**
     * Central rotational inertia of the body, usually in kg-m^2.
     */
    val inertia: Float
        get() = I + mass * (sweep.localCenter.x * sweep.localCenter.x + sweep.localCenter.y * sweep.localCenter.y)

    private val pmd = MassData()

    /**
     * Type of this body. Changing it may alter the mass and velocity.
     */
    var type: BodyType
        get() = _type
        set(type) {
            assert(!world.isLocked)
            if (world.isLocked) return

            if (_type === type) return

            _type = type

            resetMassData()

            if (_type === BodyType.STATIC) {
                _linearVelocity.setZero()
                _angularVelocity = 0.0f
                sweep.a0 = sweep.a
                sweep.c0.set(sweep.c)
                synchronizeFixtures()
            }

            isAwake = true

            force.setZero()
            torque = 0.0f

            // Delete the attached contacts.
            var ce = contactList
            while (ce != null) {
                val ce0 = ce
                ce = ce.next
                world.contactManager.destroy(ce0.contact!!)
            }
            contactList = null

            // Touch the proxies so that new contacts will be created (when appropriate)
            val broadPhase = world.contactManager.broadPhase
            var f = fixtureList
            while (f != null) {
                val proxyCount = f.proxyCount
                for (i in 0 until proxyCount) {
                    broadPhase.touchProxy(f.proxies!![i].proxyId)
                }
                f = f.next
            }
        }

    /** Whether this body treated like a bullet for continuous collision detection  */
    var isBullet: Boolean
        get() = flags and bulletFlag == bulletFlag
        set(flag) {
            flags = if (flag) {
                flags or bulletFlag
            } else {
                flags and bulletFlag.inv()
            }
        }

    /**
     * Whether this body allowed to sleep.
     * You can disable sleeping on this body. If you disable sleeping, the body will be woken.
     */
    var isSleepingAllowed: Boolean
        get() = flags and autoSleepFlag == autoSleepFlag
        set(flag) {
            if (flag) {
                flags = flags or autoSleepFlag
            } else {
                flags = flags and autoSleepFlag.inv()
                isAwake = true
            }
        }

    /**
     * Sleeping state of this body. A sleeping body has very low CPU cost.
     * Set to true to put body to sleep, false to wake it.
     */
    var isAwake: Boolean
        get() = flags and awakeFlag == awakeFlag
        set(flag) {
            if (flag) {
                if (flags and awakeFlag == 0) {
                    flags = flags or awakeFlag
                    sleepTime = 0.0f
                }
            } else {
                flags = flags and awakeFlag.inv()
                sleepTime = 0.0f
                _linearVelocity.setZero()
                _angularVelocity = 0.0f
                force.setZero()
                torque = 0.0f
            }
        }

    /**
     * Active state of the body.
     * An inactive body is not simulated and cannot be collided with or woken up.
     * If you pass a flag of true, all fixtures will be added to the broad-phase.
     * If you pass a flag of false, all fixtures will be removed from the broad-phase and
     * all contacts will be destroyed. Fixtures and joints are otherwise unaffected.
     * You may continue to create/destroy fixtures and joints on inactive bodies.
     * Fixtures on an inactive body are implicitly inactive and will not participate in collisions,
     * ray-casts, or queries. Joints connected to an inactive body are implicitly inactive.
     * An inactive body is still owned by a World object and remains in the body list.
     */
    var isActive: Boolean
        get() = flags and activeFlag == activeFlag
        set(flag) {
            assert(!world.isLocked)

            if (flag == isActive) return

            if (flag) {
                flags = flags or activeFlag

                // Create all proxies.
                val broadPhase = world.contactManager.broadPhase
                var f = fixtureList
                while (f != null) {
                    f.createProxies(broadPhase, xf)
                    f = f.next
                }
            } else {
                // Contacts are created the next time step.
                flags = flags and activeFlag.inv()

                // Destroy all proxies.
                val broadPhase = world.contactManager.broadPhase
                var f = fixtureList
                while (f != null) {
                    f.destroyProxies(broadPhase)
                    f = f.next
                }

                // Destroy the attached contacts.
                var ce = contactList
                while (ce != null) {
                    val ce0 = ce
                    ce = ce.next
                    world.contactManager.destroy(ce0.contact!!)
                }
                contactList = null
            }
        }

    /**
     * Whether this body have fixed rotation. Changing value causes the mass to be reset.
     */
    var isFixedRotation: Boolean
        get() = flags and fixedRotationFlag == fixedRotationFlag
        set(flag) {
            flags = if (flag) {
                flags or fixedRotationFlag
            } else {
                flags and fixedRotationFlag.inv()
            }
            resetMassData()
        }

    // djm pooling
    private val pxf = Transform()

    init {
        assert(bd.position.isValid)
        assert(bd.linearVelocity.isValid)
        assert(bd.gravityScale >= 0.0f)
        assert(bd.angularDamping >= 0.0f)
        assert(bd.linearDamping >= 0.0f)

        flags = 0

        if (bd.bullet) {
            flags = flags or bulletFlag
        }
        if (bd.fixedRotation) {
            flags = flags or fixedRotationFlag
        }
        if (bd.allowSleep) {
            flags = flags or autoSleepFlag
        }
        if (bd.awake) {
            flags = flags or awakeFlag
        }
        if (bd.active) {
            flags = flags or activeFlag
        }

        xf.p.set(bd.position)
        xf.q.setRadians(bd.angleRadians)

        sweep.localCenter.setZero()
        sweep.c0.set(xf.p)
        sweep.c.set(xf.p)
        sweep.a0 = bd.angleRadians
        sweep.a = bd.angleRadians
        sweep.alpha0 = 0.0f

        _linearVelocity.set(bd.linearVelocity)
        _angularVelocity = bd.angularVelocity

        linearDamping = bd.linearDamping
        angularDamping = bd.angularDamping
        gravityScale = bd.gravityScale

        force.setZero()

        _type = bd.type

        if (_type === BodyType.DYNAMIC) {
            mass = 1f
            invMass = 1f
        } else {
            mass = 0f
            invMass = 0f
        }

        userData = bd.userData
    }

    /**
     * Creates a fixture and attach it to this body. Use this function if you need to set some fixture
     * parameters, like friction. Otherwise you can create the fixture directly from a shape. If the
     * density is non-zero, this function automatically updates the mass of the body. Contacts are not
     * created until the next time step.
     *
     * @param def the fixture definition.
     * @warning This function is locked during callbacks.
     */
    fun createFixture(def: FixtureDef): Fixture? {
        assert(!world.isLocked)

        if (world.isLocked) {
            return null
        }

        val fixture = Fixture()
        fixture.create(this, def)

        if (flags and activeFlag == activeFlag) {
            val broadPhase = world.contactManager.broadPhase
            fixture.createProxies(broadPhase, xf)
        }

        fixture.next = fixtureList
        fixtureList = fixture
        ++fixtureCount

        fixture.body = this

        // Adjust mass properties if needed.
        if (fixture._density > 0.0f) {
            resetMassData()
        }

        // Let the world know we have a new fixture. This will cause new contacts
        // to be created at the beginning of the next time step.
        world.flags = world.flags or World.NEW_FIXTURE

        return fixture
    }

    /**
     * Creates a fixture from a shape and attach it to this body. This is a convenience function. Use
     * FixtureDef if you need to set parameters like friction, restitution, user data, or filtering.
     * If the density is non-zero, this function automatically updates the mass of the body.
     *
     * @param shape the shape to be cloned.
     * @param density the shape density (set to zero for static bodies).
     * @warning This function is locked during callbacks.
     */
    fun createFixture(shape: Shape, density: Float): Fixture? {
        fixDef.shape = shape
        fixDef.density = density

        return createFixture(fixDef)
    }

    /**
     * Destroy a fixture. This removes the fixture from the broad-phase and destroys all contacts
     * associated with this fixture. This will automatically adjust the mass of the body if the body
     * is dynamic and the fixture has positive density. All fixtures attached to a body are implicitly
     * destroyed when the body is destroyed.
     *
     * @param fixture the fixture to be removed.
     * @warning This function is locked during callbacks.
     */
    fun destroyFixture(fixture: Fixture) {
        assert(!world.isLocked)
        if (world.isLocked) return

        assert(fixture.body === this)

        // Remove the fixture from this body's singly linked list.
        assert(fixtureCount > 0)
        var node = fixtureList
        var last: Fixture? = null // java change
        var found = false
        while (node != null) {
            if (node === fixture) {
                node = fixture.next
                found = true
                break
            }
            last = node
            node = node.next
        }

        // You tried to remove a shape that is not attached to this body.
        assert(found)

        // java change, remove it from the list
        if (last == null) {
            fixtureList = fixture.next
        } else {
            last.next = fixture.next
        }

        // Destroy any contacts associated with the fixture.
        var edge = contactList
        while (edge != null) {
            val c = edge.contact
            edge = edge.next

            val fixtureA = c!!.fixtureA
            val fixtureB = c.fixtureB

            if (fixture === fixtureA || fixture === fixtureB) {
                // This destroys the contact and removes it from
                // this body's contact list.
                world.contactManager.destroy(c)
            }
        }

        if (flags and activeFlag == activeFlag) {
            val broadPhase = world.contactManager.broadPhase
            fixture.destroyProxies(broadPhase)
        }

        fixture.destroy()
        fixture.body = null
        fixture.next = null

        --fixtureCount

        // Reset the mass data.
        resetMassData()
    }

    /**
     * Set the position of the body's origin and rotation. This breaks any contacts and wakes the
     * other bodies. Manipulating a body's transform may cause non-physical behavior. Note: contacts
     * are updated on the next call to World.step().
     *
     * @param position the world position of the body's local origin.
     * @param angleRadians the world rotation in radians.
     */
    fun setTransformRadians(position: Vec2, angleRadians: Float) {
        assert(!world.isLocked)
        if (world.isLocked) return

        xf.q.setRadians(angleRadians)
        xf.p.set(position)

        // m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
        Transform.mulToOutUnsafe(xf, sweep.localCenter, sweep.c)
        sweep.a = angleRadians

        sweep.c0.set(sweep.c)
        sweep.a0 = sweep.a

        val broadPhase = world.contactManager.broadPhase
        var f = fixtureList
        while (f != null) {
            f.synchronize(broadPhase, xf, xf)
            f = f.next
        }
    }

    /**
     * Set the position of the body's origin and rotation. This breaks any contacts and wakes the
     * other bodies. Manipulating a body's transform may cause non-physical behavior. Note: contacts
     * are updated on the next call to World.step().
     *
     * @param position the world position of the body's local origin.
     * @param angleDegrees the world rotation in degrees.
     */
    fun setTransformDegrees(position: Vec2, angleDegrees: Float) =
        setTransformRadians(position, angleDegrees * MathUtils.DEG2RAD)

    /**
     * Set the position of the body's origin and rotation. This breaks any contacts and wakes the
     * other bodies. Manipulating a body's transform may cause non-physical behavior. Note: contacts
     * are updated on the next call to World.step().
     *
     * @param position the world position of the body's local origin.
     * @param angle the world rotation.
     */
    fun setTransform(position: Vec2, angle: Angle) = setTransformRadians(position, angle.radians.toFloat())

    /**
     * Apply a force at a world point. If the force is not applied at the center of mass, it will
     * generate a torque and affect the angular velocity. This wakes up the body.
     *
     * @param force the world force vector, usually in Newtons (N).
     * @param point the world position of the point of application.
     */
    fun applyForce(force: Vec2, point: Vec2) {
        if (_type !== BodyType.DYNAMIC) return

        if (!isAwake) {
            isAwake = true
        }

        // m_force.addLocal(force);
        // Vec2 temp = tltemp.get();
        // temp.set(point).subLocal(m_sweep.c);
        // m_torque += Vec2.cross(temp, force);

        this.force.x += force.x
        this.force.y += force.y

        torque += (point.x - sweep.c.x) * force.y - (point.y - sweep.c.y) * force.x
    }

    /**
     * Apply a force to the center of mass. This wakes up the body.
     *
     * @param force the world force vector, usually in Newtons (N).
     */
    fun applyForceToCenter(force: Vec2) {
        if (_type !== BodyType.DYNAMIC) return

        if (!isAwake) {
            isAwake = true
        }

        this.force.x += force.x
        this.force.y += force.y
    }

    /**
     * Apply a torque. This affects the angular velocity without affecting the linear velocity of the
     * center of mass. This wakes up the body.
     *
     * @param torque about the z-axis (out of the screen), usually in N-m.
     */
    fun applyTorque(torque: Float) {
        if (_type !== BodyType.DYNAMIC) return

        if (!isAwake) {
            isAwake = true
        }

        this.torque += torque
    }

    /**
     * Apply an impulse at a point. This immediately modifies the velocity. It also modifies the
     * angular velocity if the point of application is not at the center of mass. This wakes up the
     * body if 'wake' is set to true. If the body is sleeping and 'wake' is false, then there is no
     * effect.
     *
     * @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
     * @param point the world position of the point of application.
     * @param wake also wake up the body
     */
    fun applyLinearImpulse(impulse: Vec2, point: Vec2, wake: Boolean) {
        if (_type !== BodyType.DYNAMIC) return

        if (!isAwake) {
            if (wake) {
                isAwake = true
            } else {
                return
            }
        }

        _linearVelocity.x += impulse.x * invMass
        _linearVelocity.y += impulse.y * invMass

        _angularVelocity += invI * ((point.x - sweep.c.x) * impulse.y - (point.y - sweep.c.y) * impulse.x)
    }

    /**
     * Apply an angular impulse.
     *
     * @param impulse the angular impulse in units of kg*m*m/s
     */
    fun applyAngularImpulse(impulse: Float) {
        if (_type !== BodyType.DYNAMIC) return

        if (!isAwake) {
            isAwake = true
        }
        _angularVelocity += invI * impulse
    }

    /**
     * Get the mass data of the body. The rotational inertia is relative to the center of mass.
     *
     * @return a struct containing the mass, inertia and center of the body.
     */
    fun getMassData(data: MassData) {
        // data.mass = m_mass;
        // data.I = m_I + m_mass * Vec2.dot(m_sweep.localCenter, m_sweep.localCenter);
        // data.center.set(m_sweep.localCenter);

        data.mass = mass
        data.I = I + mass * (sweep.localCenter.x * sweep.localCenter.x + sweep.localCenter.y * sweep.localCenter.y)
        data.center.x = sweep.localCenter.x
        data.center.y = sweep.localCenter.y
    }

    /**
     * Set the mass properties to override the mass properties of the fixtures. Note that this changes
     * the center of mass position. Note that creating or destroying fixtures can also alter the mass.
     * This function has no effect if the body isn't dynamic.
     *
     * @param massData the mass properties.
     */
    fun setMassData(massData: MassData) {
        // TODO_ERIN adjust linear velocity and torque to account for movement of center.
        assert(!world.isLocked)
        if (world.isLocked) return

        if (_type !== BodyType.DYNAMIC) return

        invMass = 0.0f
        I = 0.0f
        invI = 0.0f

        mass = massData.mass
        if (mass <= 0.0f) {
            mass = 1f
        }

        invMass = 1.0f / mass

        if (massData.I > 0.0f && flags and fixedRotationFlag == 0) {
            I = massData.I - mass * Vec2.dot(massData.center, massData.center)
            assert(I > 0.0f)
            invI = 1.0f / I
        }

        val oldCenter = world.pool.popVec2()
        // Move center of mass.
        oldCenter.set(sweep.c)
        sweep.localCenter.set(massData.center)
        // m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
        Transform.mulToOutUnsafe(xf, sweep.localCenter, sweep.c0)
        sweep.c.set(sweep.c0)

        // Update center of mass velocity.
        // m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
        val temp = world.pool.popVec2()
        temp.set(sweep.c).subLocal(oldCenter)
        Vec2.crossToOut(_angularVelocity, temp, temp)
        _linearVelocity.addLocal(temp)

        world.pool.pushVec2(2)
    }

    /**
     * This resets the mass properties to the sum of the mass properties of the fixtures. This
     * normally does not need to be called unless you called setMassData to override the mass and you
     * later want to reset the mass.
     */
    fun resetMassData() {
        // Compute mass data from shapes. Each shape has its own density.
        mass = 0.0f
        invMass = 0.0f
        I = 0.0f
        invI = 0.0f
        sweep.localCenter.setZero()

        // Static and kinematic bodies have zero mass.
        if (_type === BodyType.STATIC || _type === BodyType.KINEMATIC) {
            // m_sweep.c0 = m_sweep.c = m_xf.position;
            sweep.c0.set(xf.p)
            sweep.c.set(xf.p)
            sweep.a0 = sweep.a
            return
        }

        assert(_type === BodyType.DYNAMIC)

        // Accumulate mass over all fixtures.
        val localCenter = world.pool.popVec2()
        localCenter.setZero()
        val temp = world.pool.popVec2()
        val massData = pmd
        var f = fixtureList
        while (f != null) {
            if (f._density == 0.0f) {
                f = f.next
                continue
            }
            f.getMassData(massData)
            mass += massData.mass
            // center += massData.mass * massData.center;
            temp.set(massData.center).mulLocal(massData.mass)
            localCenter.addLocal(temp)
            I += massData.I
            f = f.next
        }

        // Compute center of mass.
        if (mass > 0.0f) {
            invMass = 1.0f / mass
            localCenter.mulLocal(invMass)
        } else {
            // Force all dynamic bodies to have a positive mass.
            mass = 1.0f
            invMass = 1.0f
        }

        if (I > 0.0f && flags and fixedRotationFlag == 0) {
            // Center the inertia about the center of mass.
            I -= mass * Vec2.dot(localCenter, localCenter)
            assert(I > 0.0f)
            invI = 1.0f / I
        } else {
            I = 0.0f
            invI = 0.0f
        }

        val oldCenter = world.pool.popVec2()
        // Move center of mass.
        oldCenter.set(sweep.c)
        sweep.localCenter.set(localCenter)
        // m_sweep.c0 = m_sweep.c = Mul(m_xf, m_sweep.localCenter);
        Transform.mulToOutUnsafe(xf, sweep.localCenter, sweep.c0)
        sweep.c.set(sweep.c0)

        // Update center of mass velocity.
        // m_linearVelocity += Cross(m_angularVelocity, m_sweep.c - oldCenter);
        temp.set(sweep.c).subLocal(oldCenter)

        val temp2 = oldCenter
        Vec2.crossToOutUnsafe(_angularVelocity, temp, temp2)
        _linearVelocity.addLocal(temp2)

        world.pool.pushVec2(3)
    }

    /**
     * Get the world coordinates of a point given the local coordinates.
     *
     * @param localPoint a point on the body measured relative the the body's origin.
     * @return the same point expressed in world coordinates.
     */
    fun getWorldPoint(localPoint: Vec2): Vec2 {
        val v = Vec2()
        getWorldPointToOut(localPoint, v)
        return v
    }

    fun getWorldPointToOut(localPoint: Vec2, out: Vec2) {
        Transform.mulToOut(xf, localPoint, out)
    }

    /**
     * Get the world coordinates of a vector given the local coordinates.
     *
     * @param localVector a vector fixed in the body.
     * @return the same vector expressed in world coordinates.
     */
    fun getWorldVector(localVector: Vec2): Vec2 {
        val out = Vec2()
        getWorldVectorToOut(localVector, out)
        return out
    }

    fun getWorldVectorToOut(localVector: Vec2, out: Vec2) {
        Rot.mulToOut(xf.q, localVector, out)
    }

    fun getWorldVectorToOutUnsafe(localVector: Vec2, out: Vec2) {
        Rot.mulToOutUnsafe(xf.q, localVector, out)
    }

    /**
     * Gets a local point relative to the body's origin given a world point.
     *
     * @param a point in world coordinates.
     * @return the corresponding local point relative to the body's origin.
     */
    fun getLocalPoint(worldPoint: Vec2): Vec2 {
        val out = Vec2()
        getLocalPointToOut(worldPoint, out)
        return out
    }

    fun getLocalPointToOut(worldPoint: Vec2, out: Vec2) {
        Transform.mulTransToOut(xf, worldPoint, out)
    }

    /**
     * Gets a local vector given a world vector.
     *
     * @param a vector in world coordinates.
     * @return the corresponding local vector.
     */
    fun getLocalVector(worldVector: Vec2): Vec2 {
        val out = Vec2()
        getLocalVectorToOut(worldVector, out)
        return out
    }

    fun getLocalVectorToOut(worldVector: Vec2, out: Vec2) {
        Rot.mulTrans(xf.q, worldVector, out)
    }

    fun getLocalVectorToOutUnsafe(worldVector: Vec2, out: Vec2) {
        Rot.mulTransUnsafe(xf.q, worldVector, out)
    }

    /**
     * Get the world linear velocity of a world point attached to this body.
     *
     * @param a point in world coordinates.
     * @return the world velocity of a point.
     */
    fun getLinearVelocityFromWorldPoint(worldPoint: Vec2): Vec2 {
        val out = Vec2()
        getLinearVelocityFromWorldPointToOut(worldPoint, out)
        return out
    }

    fun getLinearVelocityFromWorldPointToOut(worldPoint: Vec2, out: Vec2) {
        val tempX = worldPoint.x - sweep.c.x
        val tempY = worldPoint.y - sweep.c.y
        out.x = -_angularVelocity * tempY + _linearVelocity.x
        out.y = _angularVelocity * tempX + _linearVelocity.y
    }

    /**
     * Get the world velocity of a local point.
     *
     * @param a point in local coordinates.
     * @return the world velocity of a point.
     */
    fun getLinearVelocityFromLocalPoint(localPoint: Vec2): Vec2 {
        val out = Vec2()
        getLinearVelocityFromLocalPointToOut(localPoint, out)
        return out
    }

    fun getLinearVelocityFromLocalPointToOut(localPoint: Vec2, out: Vec2) {
        getWorldPointToOut(localPoint, out)
        getLinearVelocityFromWorldPointToOut(out, out)
    }

    fun synchronizeFixtures() {
        val xf1 = pxf
        // xf1.position = m_sweep.c0 - Mul(xf1.R, m_sweep.localCenter);

        // xf1.q.set(m_sweep.a0);
        // Rot.mulToOutUnsafe(xf1.q, m_sweep.localCenter, xf1.p);
        // xf1.p.mulLocal(-1).addLocal(m_sweep.c0);
        // inlined:
        xf1.q.s = MathUtils.sin(sweep.a0)
        xf1.q.c = MathUtils.cos(sweep.a0)
        xf1.p.x = sweep.c0.x - xf1.q.c * sweep.localCenter.x + xf1.q.s * sweep.localCenter.y
        xf1.p.y = sweep.c0.y - xf1.q.s * sweep.localCenter.x - xf1.q.c * sweep.localCenter.y
        // end inline

        var f = fixtureList
        while (f != null) {
            f.synchronize(world.contactManager.broadPhase, xf1, xf)
            f = f.next
        }
    }

    fun synchronizeTransform() {
        // m_xf.q.set(m_sweep.a);
        //
        // // m_xf.position = m_sweep.c - Mul(m_xf.R, m_sweep.localCenter);
        // Rot.mulToOutUnsafe(m_xf.q, m_sweep.localCenter, m_xf.p);
        // m_xf.p.mulLocal(-1).addLocal(m_sweep.c);
        //
        xf.q.s = MathUtils.sin(sweep.a)
        xf.q.c = MathUtils.cos(sweep.a)
        val q = xf.q
        val v = sweep.localCenter
        xf.p.x = sweep.c.x - q.c * v.x + q.s * v.y
        xf.p.y = sweep.c.y - q.s * v.x - q.c * v.y
    }

    /**
     * This is used to prevent connected bodies from colliding. It may lie, depending on the
     * collideConnected flag.
     */
    fun shouldCollide(other: Body): Boolean {
        // At least one body should be dynamic.
        if (_type !== BodyType.DYNAMIC && other._type !== BodyType.DYNAMIC) {
            return false
        }

        // Does a joint prevent collision?
        var jn = jointList
        while (jn != null) {
            if (jn.other === other && !jn.joint!!.collideConnected) {
                return false
            }
            jn = jn.next
        }

        return true
    }

    fun advance(t: Float) {
        // Advance to the new safe time. This doesn't sync the broad-phase.
        sweep.advance(t)
        sweep.c.set(sweep.c0)
        sweep.a = sweep.a0
        xf.q.setRadians(sweep.a)
        // m_xf.position = m_sweep.c - Mul(m_xf.R, m_sweep.localCenter);
        Rot.mulToOutUnsafe(xf.q, sweep.localCenter, xf.p)
        xf.p.mulLocal(-1f).addLocal(sweep.c)
    }

    companion object {
        val islandFlag = 0x0001
        val awakeFlag = 0x0002
        val autoSleepFlag = 0x0004
        val bulletFlag = 0x0008
        val fixedRotationFlag = 0x0010
        val activeFlag = 0x0020
        val toiFlag = 0x0040
    }
}
