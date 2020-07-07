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

import org.jbox2d.callbacks.*
import org.jbox2d.collision.*
import org.jbox2d.collision.broadphase.*
import org.jbox2d.collision.shapes.*
import org.jbox2d.common.*
import org.jbox2d.dynamics.contacts.*
import org.jbox2d.dynamics.joints.*
import org.jbox2d.internal.*
import org.jbox2d.particle.*
import org.jbox2d.pooling.*
import org.jbox2d.pooling.arrays.*
import org.jbox2d.pooling.normal.*
import org.jbox2d.userdata.*

/**
 * The world class manages all physics entities, dynamic simulation, and asynchronous queries. The
 * world also contains efficient memory management facilities.
 *
 * @author Daniel Murphy
 */
class World(
    gravity: Vec2,
    val pool: IWorldPool,
    broadPhase: BroadPhase
) : WorldRef, Box2dTypedUserData by Box2dTypedUserData.Mixin() {

    override val world: World get() = this

    // statistics gathering
    var activeContacts = 0
    var contactPoolCount = 0

    var userData: Any? = null

    var flags: Int = CLEAR_FORCES

    /**
     * Contact manager for testing purposes
     */
    var contactManager: ContactManager = ContactManager(this, broadPhase)
        protected set

    /**
     * World body list. With the returned body, use Body.next to get the next body in the
     * world list. A null body indicates the end of the list.
     *
     * @return the head of the world body list.
     */
    var bodyList: Body? = null
        private set

    /**
     * World joint list. With the returned joint, use Joint.next to get the next joint in
     * the world list. A null joint indicates the end of the list.
     *
     * @return the head of the world joint list.
     */
    var jointList: Joint? = null
        private set

    /**
     * Number of bodies.
     */
    var bodyCount: Int = 0
        private set

    /**
     * Number of joints.
     */
    var jointCount: Int = 0
        private set

    /**
     * Global gravity vector.
     */
    var gravity = Vec2()
        set(gravity) {
            this.gravity.set(gravity)
        }

    var isSleepingAllowed: Boolean = true

    /**
     * Destruction listener. The listener is owned by you and must remain in scope.
     */
    var destructionListener: DestructionListener? = null
    var particleDestructionListener: ParticleDestructionListener? = null

    private var debugDraw: DebugDraw? = null

    /**
     * This is used to compute the time step ratio to support a variable time step.
     */
    private var inv_dt0: Float = 0f

    // these are for debugging the solver

    /**
     * Enable/disable warm starting. For testing.
     */
    var isWarmStarting: Boolean = true

    /**
     * Enable/disable continuous physics. For testing.
     */
    var isContinuousPhysics: Boolean = true

    var isSubStepping: Boolean = false

    private var stepComplete: Boolean = true

    val profile: Profile = Profile()

    private val particleSystem: ParticleSystem = ParticleSystem(this)

    private val contactStacks = Array(ShapeType.values().size) {
        arrayOfNulls<ContactRegister>(ShapeType.values().size) as Array<ContactRegister>
    }

    var isAllowSleep: Boolean
        get() = isSleepingAllowed
        set(flag) {
            if (flag == isSleepingAllowed) return

            isSleepingAllowed = flag
            if (!isSleepingAllowed) {
                var b = bodyList
                while (b != null) {
                    b.isAwake = true
                    b = b.next
                }
            }
        }

    // djm pooling
    private val step = TimeStep()
    private val stepTimer = Timer()
    private val tempTimer = Timer()

    private val color = Color3f()
    private val xf = Transform()
    private val cA = Vec2()
    private val cB = Vec2()
    private val avs = Vec2ArrayPool()

    private val wqwrapper = WorldQueryWrapper()

    private val wrcwrapper = WorldRayCastWrapper()
    private val input = RayCastInput()

    /**
     * World contact list. With the returned contact, use Contact.next to get the next
     * contact in the world list. A null contact indicates the end of the list.
     *
     * @return the head of the world contact list.
     * @warning contacts are created and destroyed in the middle of a time step. Use ContactListener
     * to avoid missing contacts.
     */
    val contactList: Contact
        get() = contactManager.contactList!!


    /**
     * Number of broad-phase proxies.
     */
    val proxyCount: Int
        get() = contactManager.broadPhase.proxyCount

    /**
     * Number of contacts (each may have 0 or more contact points).
     */
    val contactCount: Int
        get() = contactManager.contactCount

    /**
     * Height of the dynamic tree
     */
    val treeHeight: Int
        get() = contactManager.broadPhase.treeHeight

    /**
     * Balance of the dynamic tree
     */
    val treeBalance: Int
        get() = contactManager.broadPhase.treeBalance

    /**
     * Quality of the dynamic tree
     */
    val treeQuality: Float
        get() = contactManager.broadPhase.treeQuality

    /**
     * Whether the world is locked (in the middle of a time step).
     */
    val isLocked: Boolean
        get() = flags and LOCKED == LOCKED

    /**
     * Flag that controls automatic clearing of forces after each time step.
     */
    var autoClearForces: Boolean
        get() = flags and CLEAR_FORCES == CLEAR_FORCES
        set(flag) {
            flags = if (flag) {
                flags or CLEAR_FORCES
            } else {
                flags and CLEAR_FORCES.inv()
            }
        }

    private val island = Island()
    private var stack = arrayOfNulls<Body>(10) // TODO djm find a good initial stack number;
    private val broadphaseTimer = Timer()

    private val toiIsland = Island()
    private val toiInput = TimeOfImpact.TOIInput()
    private val toiOutput = TimeOfImpact.TOIOutput()
    private val subStep = TimeStep()
    private val tempBodies = arrayOfNulls<Body>(2)
    private val backup1 = Sweep()
    private val backup2 = Sweep()
    private val liquidLength = .12f
    private var averageLinearVel = -1f
    private val liquidOffset = Vec2()
    private val circCenterMoved = Vec2()
    private val liquidColor = Color3f(.4f, .4f, 1f)

    private val center = Vec2()
    private val axis = Vec2()
    private val v1 = Vec2()
    private val v2 = Vec2()
    private val tlvertices = Vec2ArrayPool()

    /**
     * Get the world particle group list. With the returned group, use ParticleGroup::next to get
     * the next group in the world list. A NULL group indicates the end of the list.
     *
     * @return the head of the world particle group list.
     */
    val particleGroupList: Array<ParticleGroup?>
        get() = particleSystem.getParticleGroupList()!!

    /**
     * Get the number of particle groups.
     */
    val particleGroupCount: Int
        get() = particleSystem.particleGroupCount

    /**
     * Get the number of particles.
     */
    val particleCount: Int
        get() = particleSystem.particleCount

    /**
     * Maximum number of particles.
     */
    var particleMaxCount: Int
        get() = particleSystem.particleMaxCount
        set(count) {
            particleSystem.particleMaxCount = count
        }

    var particleDensity: Float
        get() = particleSystem.particleDensity
        set(density) {
            particleSystem.particleDensity = density
        }

    /**
     * Particle gravity scale. Adjusts the effect of the global gravity vector on
     * particles. Default value is 1.0f.
     */
    var particleGravityScale: Float
        get() = particleSystem.particleGravityScale
        set(gravityScale) {
            particleSystem.particleGravityScale = gravityScale

        }

    /**
     * Damping for particles. Damping is used to reduce the velocity of particles.
     * The damping parameter can be larger than  1.0f but the damping effect becomes
     * sensitive to the time step when the damping parameter is large.
     */
    var particleDamping: Float
        get() = particleSystem.particleDamping
        set(damping) {
            particleSystem.particleDamping = damping
        }

    /**
     * Particle radius. You should set this only once, on world start. If you change the
     * radius during execution, existing particles may explode, shrink, or behave unexpectedly.
     */
    var particleRadius: Float
        get() = particleSystem.particleRadius
        set(radius) {
            particleSystem.particleRadius = radius
        }

    /**
     * Particle data. Returns the pointer to the head of the particle data.
     */
    val particleFlagsBuffer: IntArray
        get() = particleSystem.particleFlagsBuffer!!

    val particlePositionBuffer: Array<Vec2>
        get() = particleSystem.particlePositionBuffer!!

    val particleVelocityBuffer: Array<Vec2>
        get() = particleSystem.particleVelocityBuffer!!

    val particleColorBuffer: Array<ParticleColor>
        get() = particleSystem.particleColorBuffer!!

    val particleGroupBuffer: Array<ParticleGroup?>
        get() = particleSystem.particleGroupBuffer

    val particleUserDataBuffer: Array<Any>
        get() = particleSystem.particleUserDataBuffer!!

    /**
     * Contacts between particles
     */
    val particleContacts: Array<ParticleContact>
        get() = particleSystem.contactBuffer

    val particleContactCount: Int
        get() = particleSystem.contactCount

    /**
     * Contacts between particles and bodies
     */
    val particleBodyContacts: Array<ParticleBodyContact>
        get() = particleSystem.bodyContactBuffer

    val particleBodyContactCount: Int
        get() = particleSystem.bodyContactCount


    constructor(
        gravity: Vec2,
        pool: IWorldPool = DefaultWorldPool(WORLD_POOL_SIZE, WORLD_POOL_CONTAINER_SIZE),
        strategy: BroadPhaseStrategy = DynamicTree()
    ) : this(gravity, pool, DefaultBroadPhaseBuffer(strategy))

    init {
        this.gravity.set(gravity)
        initializeRegisters()
    }

    private fun addType(creator: IDynamicStack<Contact>, type1: ShapeType, type2: ShapeType) {
        val register = ContactRegister()
        register.creator = creator
        register.primary = true
        contactStacks[type1.ordinal][type2.ordinal] = register

        if (type1 !== type2) {
            val register2 = ContactRegister()
            register2.creator = creator
            register2.primary = false
            contactStacks[type2.ordinal][type1.ordinal] = register2
        }
    }

    private fun initializeRegisters() {
        addType(pool.circleContactStack, ShapeType.CIRCLE, ShapeType.CIRCLE)
        addType(pool.polyCircleContactStack, ShapeType.POLYGON, ShapeType.CIRCLE)
        addType(pool.polyContactStack, ShapeType.POLYGON, ShapeType.POLYGON)
        addType(pool.edgeCircleContactStack, ShapeType.EDGE, ShapeType.CIRCLE)
        addType(pool.edgePolyContactStack, ShapeType.EDGE, ShapeType.POLYGON)
        addType(pool.chainCircleContactStack, ShapeType.CHAIN, ShapeType.CIRCLE)
        addType(pool.chainPolyContactStack, ShapeType.CHAIN, ShapeType.POLYGON)
    }

    fun popContact(fixtureA: Fixture, indexA: Int, fixtureB: Fixture, indexB: Int): Contact? {
        val type1 = fixtureA.type
        val type2 = fixtureB.type

        val reg = contactStacks[type1.ordinal][type2.ordinal]
        if (reg != null) {
            if (reg.primary) {
                val c = reg.creator!!.pop()
                c.init(fixtureA, indexA, fixtureB, indexB)
                return c
            } else {
                val c = reg.creator!!.pop()
                c.init(fixtureB, indexB, fixtureA, indexA)
                return c
            }
        } else {
            return null
        }
    }

    fun pushContact(contact: Contact) {
        val fixtureA = contact.fixtureA
        val fixtureB = contact.fixtureB

        if (contact.manifold.pointCount > 0 && !fixtureA!!.isSensor && !fixtureB!!.isSensor) {
            fixtureA.body!!.isAwake = true
            fixtureB.body!!.isAwake = true
        }

        val type1 = fixtureA!!.type
        val type2 = fixtureB!!.type

        val creator = contactStacks[type1.ordinal][type2.ordinal].creator
        creator!!.push(contact)
    }

    /**
     * Register a contact filter to provide specific control over collision. Otherwise the default
     * filter is used (_defaultFilter). The listener is owned by you and must remain in scope.
     */
    fun setContactFilter(filter: ContactFilter) {
        contactManager.contactFilter = filter
    }

    /**
     * Register a contact event listener. The listener is owned by you and must remain in scope.
     */
    fun setContactListener(listener: ContactListener) {
        contactManager.contactListener = listener
    }

    /**
     * Register a routine for debug drawing. The debug draw functions are called inside with
     * World.DrawDebugData method. The debug draw object is owned by you and must remain in scope.
     */
    fun setDebugDraw(debugDraw: DebugDraw) {
        this.debugDraw = debugDraw
    }

    /**
     * Create a rigid body given a definition. No reference to the definition is retained.
     *
     * @warning This function is locked during callbacks.
     */
    fun createBody(def: BodyDef): Body {
        assert(!isLocked)
        if (isLocked) error("World is locked")
        // TODO djm pooling
        val b = Body(def, this)

        // add to world doubly linked list
        b.prev = null
        b.next = bodyList
        if (bodyList != null) {
            bodyList!!.prev = b
        }
        bodyList = b
        ++bodyCount

        return b
    }

    /**
     * Destroy a rigid body given a definition. No reference to the definition is retained. This
     * function is locked during callbacks.
     *
     * @warning This automatically deletes all associated shapes and joints.
     * @warning This function is locked during callbacks.
     */
    fun destroyBody(body: Body) {
        assert(bodyCount > 0)
        assert(!isLocked)
        if (isLocked) return

        // Delete the attached joints.
        var je = body.jointList
        while (je != null) {
            val je0 = je
            je = je.next
            if (destructionListener != null) {
                destructionListener!!.sayGoodbye(je0.joint!!)
            }

            destroyJoint(je0.joint)

            body.jointList = je
        }
        body.jointList = null

        // Delete the attached contacts.
        var ce = body.contactList
        while (ce != null) {
            val ce0 = ce
            ce = ce.next
            contactManager.destroy(ce0.contact!!)
        }
        body.contactList = null

        var f = body.fixtureList
        while (f != null) {
            val f0 = f
            f = f.next

            if (destructionListener != null) {
                destructionListener!!.sayGoodbye(f0)
            }

            f0.destroyProxies(contactManager.broadPhase)
            f0.destroy()
            // TODO djm recycle fixtures (here or in that destroy method)
            body.fixtureList = f
            body.fixtureCount -= 1
        }
        body.fixtureList = null
        body.fixtureCount = 0

        // Remove world body list.
        if (body.prev != null) {
            body.prev!!.next = body.next
        }

        if (body.next != null) {
            body.next!!.prev = body.prev
        }

        if (body == bodyList) {
            bodyList = body.next
        }

        --bodyCount
        // TODO djm recycle body
    }

    /**
     * Create a joint to constrain bodies together. No reference to the definition is retained. This
     * may cause the connected bodies to cease colliding.
     *
     * @warning This function is locked during callbacks.
     */
    fun createJoint(def: JointDef): Joint? {
        assert(!isLocked)
        if (isLocked) return null

        val j = Joint.create(this, def)

        // Connect to the world list.
        j!!.prev = null
        j.next = jointList
        if (jointList != null) {
            jointList!!.prev = j
        }
        jointList = j
        ++jointCount

        // Connect to the bodies' doubly linked lists.
        j.edgeA.joint = j
        j.edgeA.other = j.bodyB
        j.edgeA.prev = null
        j.edgeA.next = j.bodyA!!.jointList
        if (j.bodyA!!.jointList != null) {
            j.bodyA!!.jointList!!.prev = j.edgeA
        }
        j.bodyA!!.jointList = j.edgeA

        j.edgeB.joint = j
        j.edgeB.other = j.bodyA
        j.edgeB.prev = null
        j.edgeB.next = j.bodyB!!.jointList
        if (j.bodyB!!.jointList != null) {
            j.bodyB!!.jointList!!.prev = j.edgeB
        }
        j.bodyB!!.jointList = j.edgeB

        val bodyA = def.bodyA
        val bodyB = def.bodyB

        // If the joint prevents collisions, then flag any contacts for filtering.
        if (!def.collideConnected) {
            var edge = bodyB!!.contactList
            while (edge != null) {
                if (edge.other == bodyA) {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact!!.flagForFiltering()
                }

                edge = edge.next
            }
        }

        // Note: creating a joint doesn't wake the bodies.

        return j
    }

    /**
     * destroy a joint. This may cause the connected bodies to begin colliding.
     *
     * @warning This function is locked during callbacks.
     */
    fun destroyJoint(j: Joint?) {
        assert(!isLocked)
        if (isLocked) {
            return
        }

        val collideConnected = j!!.collideConnected

        // Remove from the doubly linked list.
        if (j.prev != null) {
            j.prev!!.next = j.next
        }

        if (j.next != null) {
            j.next!!.prev = j.prev
        }

        if (j === jointList) {
            jointList = j.next
        }

        // Disconnect from island graph.
        val bodyA = j.bodyA
        val bodyB = j.bodyB

        // Wake up connected bodies.
        bodyA!!.isAwake = true
        bodyB!!.isAwake = true

        // Remove from body 1.
        if (j.edgeA.prev != null) {
            j.edgeA.prev!!.next = j.edgeA.next
        }

        if (j.edgeA.next != null) {
            j.edgeA.next!!.prev = j.edgeA.prev
        }

        if (j.edgeA == bodyA.jointList) {
            bodyA.jointList = j.edgeA.next
        }

        j.edgeA.prev = null
        j.edgeA.next = null

        // Remove from body 2
        if (j.edgeB.prev != null) {
            j.edgeB.prev!!.next = j.edgeB.next
        }

        if (j.edgeB.next != null) {
            j.edgeB.next!!.prev = j.edgeB.prev
        }

        if (j.edgeB == bodyB.jointList) {
            bodyB.jointList = j.edgeB.next
        }

        j.edgeB.prev = null
        j.edgeB.next = null

        Joint.destroy(j)

        assert(jointCount > 0)
        --jointCount

        // If the joint prevents collisions, then flag any contacts for filtering.
        if (!collideConnected) {
            var edge = bodyB.contactList
            while (edge != null) {
                if (edge.other == bodyA) {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact!!.flagForFiltering()
                }

                edge = edge.next
            }
        }
    }

    /**
     * Take a time step. This performs collision detection, integration, and constraint solution.
     *
     * @param timeStep the amount of time to simulate, this should not vary.
     * @param velocityIterations for the velocity constraint solver.
     * @param positionIterations for the position constraint solver.
     */
    fun step(dt: Float, velocityIterations: Int, positionIterations: Int) {
        stepTimer.reset()
        tempTimer.reset()
        // log.debug("Starting step");
        // If new fixtures were added, we need to find the new contacts.
        if (flags and NEW_FIXTURE == NEW_FIXTURE) {
            // log.debug("There's a new fixture, lets look for new contacts");
            contactManager.findNewContacts()
            flags = flags and NEW_FIXTURE.inv()
        }

        flags = flags or LOCKED

        step.dt = dt
        step.velocityIterations = velocityIterations
        step.positionIterations = positionIterations
        if (dt > 0.0f) {
            step.invDt = 1.0f / dt
        } else {
            step.invDt = 0.0f
        }

        step.dtRatio = inv_dt0 * dt

        step.warmStarting = isWarmStarting
        profile.stepInit.record(tempTimer.milliseconds)

        // Update contacts. This is where some contacts are destroyed.
        tempTimer.reset()
        contactManager.collide()
        profile.collide.record(tempTimer.milliseconds)

        // Integrate velocities, solve velocity constraints, and integrate positions.
        if (stepComplete && step.dt > 0.0f) {
            tempTimer.reset()
            particleSystem.solve(step) // Particle Simulation
            profile.solveParticleSystem.record(tempTimer.milliseconds)
            tempTimer.reset()
            solve(step)
            profile.solve.record(tempTimer.milliseconds)
        }

        // Handle TOI events.
        if (isContinuousPhysics && step.dt > 0.0f) {
            tempTimer.reset()
            solveTOI(step)
            profile.solveTOI.record(tempTimer.milliseconds)
        }

        if (step.dt > 0.0f) {
            inv_dt0 = step.invDt
        }

        if (flags and CLEAR_FORCES == CLEAR_FORCES) {
            clearForces()
        }

        flags = flags and LOCKED.inv()
        // log.debug("ending step");

        profile.step.record(stepTimer.milliseconds)
    }

    /**
     * Call this after you are done with time steps to clear the forces. You normally call this after
     * each call to Step, unless you are performing sub-steps. By default, forces will be
     * automatically cleared, so you don't need to call this function.
     *
     * @see autoClearForces
     */
    fun clearForces() {
        var body = bodyList
        while (body != null) {
            body.force.setZero()
            body.torque = 0.0f
            body = body.next
        }
    }

    /**
     * Call this to draw shapes and other debug draw data.
     */
    fun drawDebugData() {
        if (debugDraw == null) return

        val flags = debugDraw!!.flags
        val wireframe = flags and DebugDraw.wireframeDrawingBit != 0

        if (flags and DebugDraw.shapeBit != 0) {
            var b = bodyList
            while (b != null) {
                xf.set(b.getTransform())
                var f = b.fixtureList
                while (f != null) {
                    if (!b.isActive) {
                        color.set(0.5f, 0.5f, 0.3f)
                        drawShape(f, xf, color, wireframe)
                    } else if (b.type === BodyType.STATIC) {
                        color.set(0.5f, 0.9f, 0.3f)
                        drawShape(f, xf, color, wireframe)
                    } else if (b.type === BodyType.KINEMATIC) {
                        color.set(0.5f, 0.5f, 0.9f)
                        drawShape(f, xf, color, wireframe)
                    } else if (!b.isAwake) {
                        color.set(0.5f, 0.5f, 0.5f)
                        drawShape(f, xf, color, wireframe)
                    } else {
                        color.set(0.9f, 0.7f, 0.7f)
                        drawShape(f, xf, color, wireframe)
                    }
                    f = f.next
                }
                b = b.next
            }
            drawParticleSystem(particleSystem)
        }

        if (flags and DebugDraw.jointBit != 0) {
            var j = jointList
            while (j != null) {
                drawJoint(j)
                j = j.next
            }
        }

        if (flags and DebugDraw.pairBit != 0) {
            color.set(0.3f, 0.9f, 0.9f)
            var c: Contact? = contactManager.contactList
            while (c != null) {
                val fixtureA = c.fixtureA
                val fixtureB = c.fixtureB
                fixtureA!!.getAABB(c.indexA).getCenterToOut(cA)
                fixtureB!!.getAABB(c.indexB).getCenterToOut(cB)
                debugDraw!!.drawSegment(cA, cB, color)
                c = c.next
            }
        }

        if (flags and DebugDraw.aabbBit != 0) {
            color.set(0.9f, 0.3f, 0.9f)

            var b = bodyList
            while (b != null) {
                if (b.isActive == false) {
                    b = b.next
                    continue
                }

                var f = b.fixtureList
                while (f != null) {
                    for (i in 0 until f.proxyCount) {
                        val proxy = f.proxies!![i]
                        val aabb = contactManager.broadPhase.getFatAABB(proxy.proxyId)
                        if (aabb != null) {
                            val vs = avs[4]
                            vs[0].set(aabb.lowerBound.x, aabb.lowerBound.y)
                            vs[1].set(aabb.upperBound.x, aabb.lowerBound.y)
                            vs[2].set(aabb.upperBound.x, aabb.upperBound.y)
                            vs[3].set(aabb.lowerBound.x, aabb.upperBound.y)
                            debugDraw!!.drawPolygon(vs, 4, color)
                        }
                    }
                    f = f.next
                }
                b = b.next
            }
        }

        if (flags and DebugDraw.centerOfMassBit != 0) {
            var b = bodyList
            while (b != null) {
                xf.set(b.getTransform())
                xf.p.set(b.worldCenter)
                debugDraw!!.drawTransform(xf)
                b = b.next
            }
        }

        if (flags and DebugDraw.dynamicTreeBit != 0) {
            contactManager.broadPhase.drawTree(debugDraw!!)
        }

        debugDraw!!.flush()
    }

    /**
     * Query the world for all fixtures that potentially overlap the provided AABB.
     *
     * @param callback a user implemented callback class.
     * @param aabb the query box.
     */
    fun queryAABB(callback: QueryCallback, aabb: AABB) {
        wqwrapper.broadPhase = contactManager.broadPhase
        wqwrapper.callback = callback
        contactManager.broadPhase.query(wqwrapper, aabb)
    }

    /**
     * Query the world for all fixtures and particles that potentially overlap the provided AABB.
     *
     * @param callback a user implemented callback class.
     * @param particleCallback callback for particles.
     * @param aabb the query box.
     */
    fun queryAABB(callback: QueryCallback, particleCallback: ParticleQueryCallback, aabb: AABB) {
        wqwrapper.broadPhase = contactManager.broadPhase
        wqwrapper.callback = callback
        contactManager.broadPhase.query(wqwrapper, aabb)
        particleSystem.queryAABB(particleCallback, aabb)
    }

    /**
     * Query the world for all particles that potentially overlap the provided AABB.
     *
     * @param particleCallback callback for particles.
     * @param aabb the query box.
     */
    fun queryAABB(particleCallback: ParticleQueryCallback, aabb: AABB) {
        particleSystem.queryAABB(particleCallback, aabb)
    }

    /**
     * Ray-cast the world for all fixtures in the path of the ray. Your callback controls whether you
     * get the closest point, any point, or n-points. The ray-cast ignores shapes that contain the
     * starting point.
     *
     * @param callback a user implemented callback class.
     * @param point1 the ray starting point
     * @param point2 the ray ending point
     */
    fun raycast(callback: RayCastCallback, point1: Vec2, point2: Vec2) {
        wrcwrapper.broadPhase = contactManager.broadPhase
        wrcwrapper.callback = callback
        input.maxFraction = 1.0f
        input.p1.set(point1)
        input.p2.set(point2)
        contactManager.broadPhase.raycast(wrcwrapper, input)
    }

    /**
     * Ray-cast the world for all fixtures and particles in the path of the ray. Your callback
     * controls whether you get the closest point, any point, or n-points. The ray-cast ignores shapes
     * that contain the starting point.
     *
     * @param callback a user implemented callback class.
     * @param particleCallback the particle callback class.
     * @param point1 the ray starting point
     * @param point2 the ray ending point
     */
    fun raycast(
        callback: RayCastCallback, particleCallback: ParticleRaycastCallback,
        point1: Vec2, point2: Vec2
    ) {
        wrcwrapper.broadPhase = contactManager.broadPhase
        wrcwrapper.callback = callback
        input.maxFraction = 1.0f
        input.p1.set(point1)
        input.p2.set(point2)
        contactManager.broadPhase.raycast(wrcwrapper, input)
        particleSystem.raycast(particleCallback, point1, point2)
    }

    /**
     * Ray-cast the world for all particles in the path of the ray. Your callback controls whether you
     * get the closest point, any point, or n-points.
     *
     * @param particleCallback the particle callback class.
     * @param point1 the ray starting point
     * @param point2 the ray ending point
     */
    fun raycast(particleCallback: ParticleRaycastCallback, point1: Vec2, point2: Vec2) {
        particleSystem.raycast(particleCallback, point1, point2)
    }

    private fun solve(step: TimeStep) {
        profile.solveInit.startAccum()
        profile.solveVelocity.startAccum()
        profile.solvePosition.startAccum()

        // update previous transforms
        run {
            var b = bodyList
            while (b != null) {
                b!!.xf0.set(b!!.xf)
                b = b!!.next
            }
        }

        // Size the island for the worst case.
        island.init(
            bodyCount, contactManager.contactCount, jointCount,
            contactManager.contactListener
        )

        // Clear all the island flags.
        run {
            var b = bodyList
            while (b != null) {
                b!!.flags = b!!.flags and Body.islandFlag.inv()
                b = b!!.next
            }
        }
        var c: Contact? = contactManager.contactList
        while (c != null) {
            c.flags = c.flags and Contact.ISLAND_FLAG.inv()
            c = c.next
        }
        var j = jointList
        while (j != null) {
            j.islandFlag = false
            j = j.next
        }

        // Build and simulate all awake islands.
        val stackSize = bodyCount
        if (stack.size < stackSize) {
            stack = arrayOfNulls(stackSize)
        }
        var seed = bodyList
        while (seed != null) {
            if (seed.flags and Body.islandFlag == Body.islandFlag) {
                seed = seed.next
                continue
            }

            if (seed.isAwake == false || seed.isActive == false) {
                seed = seed.next
                continue
            }

            // The seed can be dynamic or kinematic.
            if (seed.type === BodyType.STATIC) {
                seed = seed.next
                continue
            }

            // Reset island and stack.
            island.clear()
            var stackCount = 0
            stack[stackCount++] = seed
            seed.flags = seed.flags or Body.islandFlag

            // Perform a depth first search (DFS) on the constraint graph.
            while (stackCount > 0) {
                // Grab the next body off the stack and add it to the island.
                val b = stack[--stackCount]!!
                assert(b.isActive == true)
                island.add(b)

                // Make sure the body is awake.
                b.isAwake = true

                // To keep islands as small as possible, we don't
                // propagate islands across static bodies.
                if (b.type === BodyType.STATIC) {
                    continue
                }

                // Search all contacts connected to this body.
                var ce = b.contactList
                while (ce != null) {
                    val contact = ce.contact

                    // Has this contact already been added to an island?
                    if (contact!!.flags and Contact.ISLAND_FLAG == Contact.ISLAND_FLAG) {
                        ce = ce.next
                        continue
                    }

                    // Is this contact solid and touching?
                    if (contact.isEnabled == false || contact.isTouching == false) {
                        ce = ce.next
                        continue
                    }

                    // Skip sensors.
                    val sensorA = contact.fixtureA!!._isSensor
                    val sensorB = contact.fixtureB!!._isSensor
                    if (sensorA || sensorB) {
                        ce = ce.next
                        continue
                    }

                    island.add(contact)
                    contact.flags = contact.flags or Contact.ISLAND_FLAG

                    val other = ce.other

                    // Was the other body already added to this island?
                    if (other!!.flags and Body.islandFlag == Body.islandFlag) {
                        ce = ce.next
                        continue
                    }

                    assert(stackCount < stackSize)
                    stack[stackCount++] = other
                    other.flags = other.flags or Body.islandFlag
                    ce = ce.next
                }

                // Search all joints connect to this body.
                var je = b.jointList
                while (je != null) {
                    if (je.joint!!.islandFlag) {
                        je = je.next
                        continue
                    }

                    val other = je.other

                    // Don't simulate joints connected to inactive bodies.
                    if (!other!!.isActive) {
                        je = je.next
                        continue
                    }

                    island.add(je.joint!!)
                    je.joint!!.islandFlag = true

                    if (other.flags and Body.islandFlag == Body.islandFlag) {
                        je = je.next
                        continue
                    }

                    assert(stackCount < stackSize)
                    stack[stackCount++] = other
                    other.flags = other.flags or Body.islandFlag
                    je = je.next
                }
            }
            island.solve(profile, step, gravity, isSleepingAllowed)

            // Post solve cleanup.
            for (i in 0 until island.bodyCount) {
                // Allow static bodies to participate in other islands.
                val b = island.bodies!![i]
                if (b.type === BodyType.STATIC) {
                    b.flags = b.flags and Body.islandFlag.inv()
                }
            }
            seed = seed.next
        }
        profile.solveInit.endAccum()
        profile.solveVelocity.endAccum()
        profile.solvePosition.endAccum()

        broadphaseTimer.reset()
        // Synchronize fixtures, check for out of range bodies.
        var b = bodyList
        while (b != null) {
            // If a body was not in an island then it did not move.
            if (b!!.flags and Body.islandFlag == 0) {
                b = b!!.next
                continue
            }

            if (b!!.type === BodyType.STATIC) {
                b = b!!.next
                continue
            }

            // Update fixtures (for broad-phase).
            b!!.synchronizeFixtures()
            b = b!!.next
        }

        // Look for new contacts.
        contactManager.findNewContacts()
        profile.broadphase.record(broadphaseTimer.milliseconds)
    }

    private fun solveTOI(step: TimeStep) {
        val island = toiIsland
        island.init(
            2 * Settings.maxTOIContacts, Settings.maxTOIContacts, 0,
            contactManager.contactListener
        )
        if (stepComplete) {
            var b = bodyList
            while (b != null) {
                b.flags = b.flags and Body.islandFlag.inv()
                b.sweep.alpha0 = 0.0f
                b = b.next
            }

            var c: Contact? = contactManager.contactList
            while (c != null) {
                // Invalidate TOI
                c.flags = c.flags and (Contact.TOI_FLAG or Contact.ISLAND_FLAG).inv()
                c.toiCount = 0f
                c.toi = 1.0f
                c = c.next
            }
        }

        // Find TOI events and solve them.
        while (true) {
            // Find the first TOI.
            var minContact: Contact? = null
            var minAlpha = 1.0f

            var c: Contact? = contactManager.contactList
            while (c != null) {
                // Is this contact disabled?
                if (c.isEnabled == false) {
                    c = c.next
                    continue
                }

                // Prevent excessive sub-stepping.
                if (c.toiCount > Settings.maxSubSteps) {
                    c = c.next
                    continue
                }

                var alpha = 1.0f
                if (c.flags and Contact.TOI_FLAG != 0) {
                    // This contact has a valid cached TOI.
                    alpha = c.toi
                } else {
                    val fA = c.fixtureA
                    val fB = c.fixtureB

                    // Is there a sensor?
                    if (fA!!.isSensor || fB!!.isSensor) {
                        c = c.next
                        continue
                    }

                    val bA = fA.body
                    val bB = fB.body

                    val typeA = bA!!._type
                    val typeB = bB!!._type
                    assert(typeA === BodyType.DYNAMIC || typeB === BodyType.DYNAMIC)

                    val activeA = bA.isAwake && typeA !== BodyType.STATIC
                    val activeB = bB.isAwake && typeB !== BodyType.STATIC

                    // Is at least one body active (awake and dynamic or kinematic)?
                    if (activeA == false && activeB == false) {
                        c = c.next
                        continue
                    }

                    val collideA = bA.isBullet || typeA !== BodyType.DYNAMIC
                    val collideB = bB.isBullet || typeB !== BodyType.DYNAMIC

                    // Are these two non-bullet dynamic bodies?
                    if (collideA == false && collideB == false) {
                        c = c.next
                        continue
                    }

                    // Compute the TOI for this contact.
                    // Put the sweeps onto the same time interval.
                    var alpha0 = bA.sweep.alpha0

                    if (bA.sweep.alpha0 < bB.sweep.alpha0) {
                        alpha0 = bB.sweep.alpha0
                        bA.sweep.advance(alpha0)
                    } else if (bB.sweep.alpha0 < bA.sweep.alpha0) {
                        alpha0 = bA.sweep.alpha0
                        bB.sweep.advance(alpha0)
                    }

                    assert(alpha0 < 1.0f)

                    val indexA = c.indexA
                    val indexB = c.indexB

                    // Compute the time of impact in interval [0, minTOI]
                    val input = toiInput
                    input.proxyA.set(fA.shape!!, indexA)
                    input.proxyB.set(fB.shape!!, indexB)
                    input.sweepA.set(bA.sweep)
                    input.sweepB.set(bB.sweep)
                    input.tMax = 1.0f

                    pool.timeOfImpact.timeOfImpact(toiOutput, input)

                    // Beta is the fraction of the remaining portion of the .
                    val beta = toiOutput.t
                    if (toiOutput.state === TimeOfImpact.TOIOutputState.TOUCHING) {
                        alpha = MathUtils.min(alpha0 + (1.0f - alpha0) * beta, 1.0f)
                    } else {
                        alpha = 1.0f
                    }

                    c.toi = alpha
                    c.flags = c.flags or Contact.TOI_FLAG
                }

                if (alpha < minAlpha) {
                    // This is the minimum TOI found so far.
                    minContact = c
                    minAlpha = alpha
                }
                c = c.next
            }

            if (minContact == null || 1.0f - 10.0f * Settings.EPSILON < minAlpha) {
                // No more TOI events. Done!
                stepComplete = true
                break
            }

            // Advance the bodies to the TOI.
            val fA = minContact.fixtureA
            val fB = minContact.fixtureB
            val bA = fA!!.body
            val bB = fB!!.body

            backup1.set(bA!!.sweep)
            backup2.set(bB!!.sweep)

            bA.advance(minAlpha)
            bB.advance(minAlpha)

            // The TOI contact likely has some new contact points.
            minContact.update(contactManager.contactListener)
            minContact.flags = minContact.flags and Contact.TOI_FLAG.inv()
            ++minContact.toiCount

            // Is the contact solid?
            if (minContact.isEnabled == false || minContact.isTouching == false) {
                // Restore the sweeps.
                minContact.isEnabled = false
                bA.sweep.set(backup1)
                bB.sweep.set(backup2)
                bA.synchronizeTransform()
                bB.synchronizeTransform()
                continue
            }

            bA.isAwake = true
            bB.isAwake = true

            // Build the island
            island.clear()
            island.add(bA)
            island.add(bB)
            island.add(minContact)

            bA.flags = bA.flags or Body.islandFlag
            bB.flags = bB.flags or Body.islandFlag
            minContact.flags = minContact.flags or Contact.ISLAND_FLAG

            // Get contacts on bodyA and bodyB.
            tempBodies[0] = bA
            tempBodies[1] = bB
            for (i in 0..1) {
                val body = tempBodies[i]!!
                if (body._type === BodyType.DYNAMIC) {
                    var ce = body.contactList
                    while (ce != null) {
                        if (island.bodyCount == island.bodyCapacity) {
                            break
                        }

                        if (island.contactCount == island.contactCapacity) {
                            break
                        }

                        val contact = ce.contact

                        // Has this contact already been added to the island?
                        if (contact!!.flags and Contact.ISLAND_FLAG != 0) {
                            ce = ce.next
                            continue
                        }

                        // Only add static, kinematic, or bullet bodies.
                        val other = ce.other
                        if (other!!._type === BodyType.DYNAMIC && !body.isBullet && !other!!.isBullet) {
                            ce = ce.next
                            continue
                        }

                        // Skip sensors.
                        val sensorA = contact.fixtureA!!._isSensor
                        val sensorB = contact.fixtureB!!._isSensor
                        if (sensorA || sensorB) {
                            ce = ce.next
                            continue
                        }

                        // Tentatively advance the body to the TOI.
                        backup1.set(other!!.sweep)
                        if (other.flags and Body.islandFlag == 0) {
                            other.advance(minAlpha)
                        }

                        // Update the contact points
                        contact.update(contactManager.contactListener)

                        // Was the contact disabled by the user?
                        if (!contact.isEnabled) {
                            other.sweep.set(backup1)
                            other.synchronizeTransform()
                            ce = ce.next
                            continue
                        }

                        // Are there contact points?
                        if (!contact.isTouching) {
                            other.sweep.set(backup1)
                            other.synchronizeTransform()
                            ce = ce.next
                            continue
                        }

                        // Add the contact to the island
                        contact.flags = contact.flags or Contact.ISLAND_FLAG
                        island.add(contact)

                        // Has the other body already been added to the island?
                        if (other.flags and Body.islandFlag != 0) {
                            ce = ce.next
                            continue
                        }

                        // Add the other body to the island.
                        other.flags = other.flags or Body.islandFlag

                        if (other._type !== BodyType.STATIC) {
                            other.isAwake = true
                        }

                        island.add(other)
                        ce = ce.next
                    }
                }
            }

            subStep.dt = (1.0f - minAlpha) * step.dt
            subStep.invDt = 1.0f / subStep.dt
            subStep.dtRatio = 1.0f
            subStep.positionIterations = 20
            subStep.velocityIterations = step.velocityIterations
            subStep.warmStarting = false
            island.solveTOI(subStep, bA.islandIndex, bB.islandIndex)

            // Reset island flags and synchronize broad-phase proxies.
            for (i in 0 until island.bodyCount) {
                val body = island.bodies!![i]
                body.flags = body.flags and Body.islandFlag.inv()

                if (body._type !== BodyType.DYNAMIC) {
                    continue
                }

                body.synchronizeFixtures()

                // Invalidate all contact TOIs on this displaced body.
                var ce = body.contactList
                while (ce != null) {
                    ce.contact!!.flags = ce.contact!!.flags and (Contact.TOI_FLAG or Contact.ISLAND_FLAG).inv()
                    ce = ce.next
                }
            }

            // Commit fixture proxy movements to the broad-phase so that new contacts are created.
            // Also, some contacts can be destroyed.
            contactManager.findNewContacts()

            if (isSubStepping) {
                stepComplete = false
                break
            }
        }
    }

    private fun drawJoint(joint: Joint) {
        val bodyA = joint.bodyA
        val bodyB = joint.bodyB
        val xf1 = bodyA!!.getTransform()
        val xf2 = bodyB!!.getTransform()
        val x1 = xf1.p
        val x2 = xf2.p
        val p1 = pool.popVec2()
        val p2 = pool.popVec2()
        joint.getAnchorA(p1)
        joint.getAnchorB(p2)

        color.set(0.5f, 0.8f, 0.8f)

        when (joint.type) {
            // TODO djm write after writing joints
            JointType.DISTANCE -> debugDraw!!.drawSegment(p1, p2, color)

            JointType.PULLEY -> {
                val pulley = joint as PulleyJoint
                val s1 = pulley.groundAnchorA
                val s2 = pulley.groundAnchorB
                debugDraw!!.drawSegment(s1, p1, color)
                debugDraw!!.drawSegment(s2, p2, color)
                debugDraw!!.drawSegment(s1, s2, color)
            }
            JointType.CONSTANT_VOLUME, JointType.MOUSE -> Unit
            else -> {
                debugDraw!!.drawSegment(x1, p1, color)
                debugDraw!!.drawSegment(p1, p2, color)
                debugDraw!!.drawSegment(x2, p2, color)
            }
        }// don't draw this
        pool.pushVec2(2)
    }

    private fun drawShape(fixture: Fixture, xf: Transform, color: Color3f, wireframe: Boolean) {
        when (fixture.type) {
            ShapeType.CIRCLE -> {
                val circle = fixture.shape as CircleShape

                // Vec2 center = Mul(xf, circle.m_p);
                Transform.mulToOutUnsafe(xf, circle.p, center)
                val radius = circle.radius
                xf.q.getXAxis(axis)

                if (fixture.userData != null && fixture.userData == LIQUID_INT) {
                    val b = fixture.body
                    liquidOffset.set(b!!._linearVelocity)
                    val linVelLength = b._linearVelocity.length()
                    if (averageLinearVel == -1f) {
                        averageLinearVel = linVelLength
                    } else {
                        averageLinearVel = .98f * averageLinearVel + .02f * linVelLength
                    }
                    liquidOffset.mulLocal(liquidLength / averageLinearVel / 2f)
                    circCenterMoved.set(center).addLocal(liquidOffset)
                    center.subLocal(liquidOffset)
                    debugDraw!!.drawSegment(center, circCenterMoved, liquidColor)
                    return
                }
                if (wireframe) {
                    debugDraw!!.drawCircle(center, radius, axis, color)
                } else {
                    debugDraw!!.drawSolidCircle(center, radius, axis, color)
                }
            }

            ShapeType.POLYGON -> {
                val poly = fixture.shape as PolygonShape
                val vertexCount = poly.vertexCount
                assert(vertexCount <= Settings.maxPolygonVertices)
                val vertices = tlvertices[Settings.maxPolygonVertices]

                for (i in 0 until vertexCount) {
                    // vertices[i] = Mul(xf, poly.m_vertices[i]);
                    Transform.mulToOutUnsafe(xf, poly.vertices[i], vertices[i])
                }
                if (wireframe) {
                    debugDraw!!.drawPolygon(vertices, vertexCount, color)
                } else {
                    debugDraw!!.drawSolidPolygon(vertices, vertexCount, color)
                }
            }
            ShapeType.EDGE -> {
                val edge = fixture.shape as EdgeShape
                Transform.mulToOutUnsafe(xf, edge.vertex1, v1)
                Transform.mulToOutUnsafe(xf, edge.vertex2, v2)
                debugDraw!!.drawSegment(v1, v2, color)
            }
            ShapeType.CHAIN -> {
                val chain = fixture.shape as ChainShape
                val count = chain.count
                val vertices = chain.vertices

                Transform.mulToOutUnsafe(xf, vertices!![0], v1)
                for (i in 1 until count) {
                    Transform.mulToOutUnsafe(xf, vertices[i], v2)
                    debugDraw!!.drawSegment(v1, v2, color)
                    debugDraw!!.drawCircle(v1, 0.05f, color)
                    v1.set(v2)
                }
            }
            else -> {
            }
        }
    }

    private fun drawParticleSystem(system: ParticleSystem) {
        val wireframe = debugDraw!!.flags and DebugDraw.wireframeDrawingBit != 0
        val particleCount = system.particleCount
        if (particleCount != 0) {
            val particleRadius = system.particleRadius
            val positionBuffer = system.particlePositionBuffer
            var colorBuffer: Array<ParticleColor>? = null
            if (system.colorBuffer.data != null) {
                colorBuffer = system.particleColorBuffer
            }
            if (wireframe) {
                debugDraw!!.drawParticlesWireframe(
                    positionBuffer!!, particleRadius, colorBuffer!!,
                    particleCount
                )
            } else {
                debugDraw!!.drawParticles(positionBuffer!!, particleRadius, colorBuffer!!, particleCount)
            }
        }
    }

    /**
     * Create a particle whose properties have been defined. No reference to the definition is
     * retained. A simulation step must occur before it's possible to interact with a newly created
     * particle. For example, DestroyParticleInShape() will not destroy a particle until Step() has
     * been called.
     *
     * @warning This function is locked during callbacks.
     * @return the index of the particle.
     */
    fun createParticle(def: ParticleDef): Int {
        assert(!isLocked)
        if (isLocked) return 0
        val p = particleSystem.createParticle(def)
        return p
    }

    /**
     * Destroy a particle. The particle is removed after the next step.
     *
     * @param Index of the particle to destroy.
     * @param Whether to call the destruction listener just before the particle is destroyed.
     */
    fun destroyParticle(index: Int, callDestructionListener: Boolean = false) {
        particleSystem.destroyParticle(index, callDestructionListener)
    }

    /**
     * Destroy particles inside a shape. This function is locked during callbacks. In addition, this
     * function immediately destroys particles in the shape in contrast to DestroyParticle() which
     * defers the destruction until the next simulation step.
     *
     * @param Shape which encloses particles that should be destroyed.
     * @param Transform applied to the shape.
     * @param Whether to call the world b2DestructionListener for each particle destroyed.
     * @warning This function is locked during callbacks.
     * @return Number of particles destroyed.
     */
    fun destroyParticlesInShape(shape: Shape, xf: Transform, callDestructionListener: Boolean = false): Int {
        assert(!isLocked)
        return if (isLocked) {
            0
        } else particleSystem.destroyParticlesInShape(shape, xf, callDestructionListener)
    }

    /**
     * Create a particle group whose properties have been defined. No reference to the definition is
     * retained.
     *
     * @warning This function is locked during callbacks.
     */
    fun createParticleGroup(def: ParticleGroupDef): ParticleGroup? {
        assert(!isLocked)
        if (isLocked) {
            return null
        }
        val g = particleSystem.createParticleGroup(def)
        return g
    }

    /**
     * Join two particle groups.
     *
     * @param the first group. Expands to encompass the second group.
     * @param the second group. It is destroyed.
     * @warning This function is locked during callbacks.
     */
    fun joinParticleGroups(groupA: ParticleGroup, groupB: ParticleGroup) {
        assert(!isLocked)
        if (isLocked) {
            return
        }
        particleSystem.joinParticleGroups(groupA, groupB)
    }

    /**
     * Destroy particles in a group. This function is locked during callbacks.
     *
     * @param The particle group to destroy.
     * @param Whether to call the world b2DestructionListener for each particle is destroyed.
     * @warning This function is locked during callbacks.
     */
    fun destroyParticlesInGroup(group: ParticleGroup, callDestructionListener: Boolean = false) {
        assert(!isLocked)
        if (isLocked) return
        particleSystem.destroyParticlesInGroup(group, callDestructionListener)
    }

    /**
     * Set a buffer for particle data.
     *
     * @param buffer is a pointer to a block of memory.
     * @param size is the number of values in the block.
     */
    fun setParticleFlagsBuffer(buffer: IntArray, capacity: Int) {
        particleSystem.setParticleFlagsBuffer(buffer, capacity)
    }

    fun setParticlePositionBuffer(buffer: Array<Vec2>, capacity: Int) {
        particleSystem.setParticlePositionBuffer(buffer, capacity)
    }

    fun setParticleVelocityBuffer(buffer: Array<Vec2>, capacity: Int) {
        particleSystem.setParticleVelocityBuffer(buffer, capacity)
    }

    fun setParticleColorBuffer(buffer: Array<ParticleColor>, capacity: Int) {
        particleSystem.setParticleColorBuffer(buffer, capacity)
    }

    fun setParticleUserDataBuffer(buffer: Array<Any>, capacity: Int) {
        particleSystem.setParticleUserDataBuffer(buffer, capacity)
    }

    /**
     * Compute the kinetic energy that can be lost by damping force
     */
    fun computeParticleCollisionEnergy(): Float {
        return particleSystem.computeParticleCollisionEnergy()
    }

    companion object {
        const val WORLD_POOL_SIZE = 100
        const val WORLD_POOL_CONTAINER_SIZE = 10

        const val NEW_FIXTURE = 0x0001
        const val LOCKED = 0x0002
        const val CLEAR_FORCES = 0x0004

        // NOTE this corresponds to the liquid test, so the debugdraw can draw
        // the liquid particles correctly. They should be the same.
        private const val LIQUID_INT = 1234598372
    }
}

internal class WorldQueryWrapper : TreeCallback {

    var broadPhase: BroadPhase? = null
    var callback: QueryCallback? = null

    override fun treeCallback(proxyId: Int): Boolean {
        val proxy = broadPhase!!.getUserData(proxyId) as FixtureProxy?
        return callback!!.reportFixture(proxy!!.fixture!!)
    }
}

internal class WorldRayCastWrapper : TreeRayCastCallback {

    // djm pooling
    private val output = RayCastOutput()
    private val temp = Vec2()
    private val point = Vec2()

    var broadPhase: BroadPhase? = null
    var callback: RayCastCallback? = null

    override fun raycastCallback(input: RayCastInput, nodeId: Int): Float {
        val userData = broadPhase!!.getUserData(nodeId)
        val proxy = userData as FixtureProxy?
        val fixture = proxy!!.fixture
        val index = proxy.childIndex
        val hit = fixture!!.raycast(output, input, index)

        if (hit) {
            val fraction = output.fraction
            // Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
            temp.set(input.p2).mulLocal(fraction)
            point.set(input.p1).mulLocal(1 - fraction).addLocal(temp)
            return callback!!.reportFixture(fixture, point, output.normal, fraction)
        }

        return input.maxFraction
    }
}
