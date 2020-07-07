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

import org.jbox2d.callbacks.ContactImpulse
import org.jbox2d.callbacks.ContactListener
import org.jbox2d.common.MathUtils
import org.jbox2d.common.Settings
import org.jbox2d.common.Timer
import org.jbox2d.common.Vec2
import org.jbox2d.dynamics.contacts.Contact
import org.jbox2d.dynamics.contacts.ContactSolver
import org.jbox2d.dynamics.contacts.ContactSolver.ContactSolverDef
import org.jbox2d.dynamics.contacts.ContactVelocityConstraint
import org.jbox2d.dynamics.contacts.Position
import org.jbox2d.dynamics.contacts.Velocity
import org.jbox2d.dynamics.joints.Joint
import org.jbox2d.internal.*

/*
 Position Correction Notes
 =========================
 I tried the several algorithms for position correction of the 2D revolute joint.
 I looked at these systems:
 - simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
 - suspension bridge with 30 1m long planks of length 1m.
 - multi-link chain with 30 1m long links.

 Here are the algorithms:

 Baumgarte - A fraction of the position error is added to the velocity error. There is no
 separate position solver.

 Pseudo Velocities - After the velocity solver and position integration,
 the position error, Jacobian, and effective mass are recomputed. Then
 the velocity constraints are solved with pseudo velocities and a fraction
 of the position error is added to the pseudo velocity error. The pseudo
 velocities are initialized to zero and there is no warm-starting. After
 the position solver, the pseudo velocities are added to the positions.
 This is also called the First Order World method or the Position LCP method.

 Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
 position error is re-computed for each raint and the positions are updated
 after the raint is solved. The radius vectors (aka Jacobians) are
 re-computed too (otherwise the algorithm has horrible instability). The pseudo
 velocity states are not needed because they are effectively zero at the beginning
 of each iteration. Since we have the current position error, we allow the
 iterations to terminate early if the error becomes smaller than Settings.linearSlop.

 Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
 each time a raint is solved.

 Here are the results:
 Baumgarte - this is the cheapest algorithm but it has some stability problems,
 especially with the bridge. The chain links separate easily close to the root
 and they jitter as they struggle to pull together. This is one of the most common
 methods in the field. The big drawback is that the position correction artificially
 affects the momentum, thus leading to instabilities and false bounce. I used a
 bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
 factor makes joints and contacts more spongy.

 Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
 stable. However, joints still separate with large angular velocities. Drag the
 simple pendulum in a circle quickly and the joint will separate. The chain separates
 easily and does not recover. I used a bias factor of 0.2. A larger value lead to
 the bridge collapsing when a heavy cube drops on it.

 Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
 Velocities, but in other ways it is worse. The bridge and chain are much more
 stable, but the simple pendulum goes unstable at high angular velocities.

 Full NGS - stable in all tests. The joints display good stiffness. The bridge
 still sags, but this is better than infinite forces.

 Recommendations
 Pseudo Velocities are not really worthwhile because the bridge and chain cannot
 recover from joint separation. In other cases the benefit over Baumgarte is small.

 Modified NGS is not a robust method for the revolute joint due to the violent
 instability seen in the simple pendulum. Perhaps it is viable with other raint
 types, especially scalar constraints where the effective mass is a scalar.

 This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
 and is very fast. I don't think we can escape Baumgarte, especially in highly
 demanding cases where high raint fidelity is not needed.

 Full NGS is robust and easy on the eyes. I recommend this as an option for
 higher fidelity simulation and certainly for suspension bridges and long chains.
 Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
 joint separation can be problematic. The number of NGS iterations can be reduced
 for better performance without harming robustness much.

 Each joint in a can be handled differently in the position solver. So I recommend
 a system where the user can select the algorithm on a per joint basis. I would
 probably default to the slower Full NGS and let the user select the faster
 Baumgarte method in performance critical scenarios.
 */

/*
 Cache Performance

 The Box2D solvers are dominated by cache misses. Data structures are designed
 to increase the number of cache hits. Much of misses are due to random access
 to body data. The raint structures are iterated over linearly, which leads
 to few cache misses.

 The bodies are not accessed during iteration. Instead read only data, such as
 the mass values are stored with the constraints. The mutable data are the raint
 impulses and the bodies velocities/positions. The impulses are held inside the
 raint structures. The body velocities/positions are held in compact, temporary
 arrays to increase the number of cache hits. Linear and angular velocity are
 stored in a single array since multiple arrays lead to multiple misses.
 */

/*
 2D Rotation

 R = [cos(theta) -sin(theta)]
 [sin(theta) cos(theta) ]

 thetaDot = omega

 Let q1 = cos(theta), q2 = sin(theta).
 R = [q1 -q2]
 [q2  q1]

 q1Dot = -thetaDot * q2
 q2Dot = thetaDot * q1

 q1_new = q1_old - dt * w * q2
 q2_new = q2_old + dt * w * q1
 then normalize.

 This might be faster than computing sin+cos.
 However, we can compute sin+cos of the same angle fast.
 */

/**
 * This is an internal class.
 *
 * @author Daniel Murphy
 */
class Island {

    var listener: ContactListener? = null

    var bodies: Array<Body>? = null
    var contacts: Array<Contact>? = null
    var joints: Array<Joint>? = null

    var positions: Array<Position>? = null
    var velocities: Array<Velocity>? = null

    var bodyCount: Int = 0
    var jointCount: Int = 0
    var contactCount: Int = 0

    var bodyCapacity: Int = 0
    var contactCapacity: Int = 0
    var jointCapacity: Int = 0

    private val contactSolver = ContactSolver()
    private val timer = Timer()
    private val solverData = SolverData()
    private val solverDef = ContactSolverDef()

    private val toiContactSolver = ContactSolver()
    private val toiSolverDef = ContactSolverDef()

    private val impulse = ContactImpulse()

    fun init(bodyCapacity: Int, contactCapacity: Int, jointCapacity: Int, listener: ContactListener?) {
        // System.out.println("Initializing Island");
        this.bodyCapacity = bodyCapacity
        this.contactCapacity = contactCapacity
        this.jointCapacity = jointCapacity
        bodyCount = 0
        contactCount = 0
        jointCount = 0

        this.listener = listener

        if (bodies == null || this.bodyCapacity > bodies!!.size) {
            bodies = arrayOfNulls<Body>(this.bodyCapacity) as Array<Body>
        }
        if (joints == null || this.jointCapacity > joints!!.size) {
            joints = arrayOfNulls<Joint>(this.jointCapacity) as Array<Joint>
        }
        if (contacts == null || this.contactCapacity > contacts!!.size) {
            contacts = arrayOfNulls<Contact>(this.contactCapacity) as Array<Contact>
        }

        // dynamic array
        if (velocities == null || this.bodyCapacity > velocities!!.size) {
            val old = if (velocities == null) emptyArray<Velocity>() else velocities
            velocities = arrayOfNulls<Velocity>(this.bodyCapacity) as Array<Velocity>
            arraycopy(old!!, 0, velocities!!, 0, old!!.size)
            for (i in old!!.size until velocities!!.size) {
                velocities!![i] = Velocity()
            }
        }

        // dynamic array
        if (positions == null || this.bodyCapacity > positions!!.size) {
            val old = if (positions == null) emptyArray() else positions
            positions = arrayOfNulls<Position>(this.bodyCapacity) as Array<Position>
            arraycopy(old!!, 0, positions!!, 0, old.size)
            for (i in old.size until positions!!.size) {
                positions!![i] = Position()
            }
        }
    }

    fun clear() {
        bodyCount = 0
        contactCount = 0
        jointCount = 0
    }

    fun solve(profile: Profile, step: TimeStep, gravity: Vec2, allowSleep: Boolean) {
        // System.out.println("Solving Island");
        val h = step.dt

        // Integrate velocities and apply damping. Initialize the body state.
        for (i in 0 until bodyCount) {
            val b = bodies!![i]
            val bm_sweep = b.sweep
            val c = bm_sweep.c
            val a = bm_sweep.a
            val v = b._linearVelocity
            var w = b._angularVelocity

            // Store positions for continuous collision.
            bm_sweep.c0.set(bm_sweep.c)
            bm_sweep.a0 = bm_sweep.a

            if (b._type === BodyType.DYNAMIC) {
                // Integrate velocities.
                // v += h * (b.m_gravityScale * gravity + b.m_invMass * b.m_force);
                v.x += h * (b.gravityScale * gravity.x + b.invMass * b.force.x)
                v.y += h * (b.gravityScale * gravity.y + b.invMass * b.force.y)
                w += h * b.invI * b.torque

                // Apply damping.
                // ODE: dv/dt + c * v = 0
                // Solution: v(t) = v0 * exp(-c * t)
                // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v *
                // exp(-c * dt)
                // v2 = exp(-c * dt) * v1
                // Pade approximation:
                // v2 = v1 * 1 / (1 + c * dt)
                v.x *= 1.0f / (1.0f + h * b.linearDamping)
                v.y *= 1.0f / (1.0f + h * b.linearDamping)
                w *= 1.0f / (1.0f + h * b.angularDamping)
            }

            positions!![i].c.x = c.x
            positions!![i].c.y = c.y
            positions!![i].a = a
            velocities!![i].v.x = v.x
            velocities!![i].v.y = v.y
            velocities!![i].w = w
        }

        timer.reset()

        // Solver data
        solverData.step = step
        solverData.positions = positions
        solverData.velocities = velocities

        // Initialize velocity constraints.
        solverDef.step = step
        solverDef.contacts = contacts
        solverDef.count = contactCount
        solverDef.positions = positions
        solverDef.velocities = velocities

        contactSolver.init(solverDef)
        // System.out.println("island init vel");
        contactSolver.initializeVelocityConstraints()

        if (step.warmStarting) {
            // System.out.println("island warm start");
            contactSolver.warmStart()
        }

        for (i in 0 until jointCount) {
            joints!![i].initVelocityConstraints(solverData)
        }

        profile.solveInit.accum(timer.milliseconds)

        // Solve velocity constraints
        timer.reset()
        // System.out.println("island solving velocities");
        for (i in 0 until step.velocityIterations) {
            for (j in 0 until jointCount) {
                joints!![j].solveVelocityConstraints(solverData)
            }

            contactSolver.solveVelocityConstraints()
        }

        // Store impulses for warm starting
        contactSolver.storeImpulses()
        profile.solveVelocity.accum(timer.milliseconds)

        // Integrate positions
        for (i in 0 until bodyCount) {
            val c = positions!![i].c
            var a = positions!![i].a
            val v = velocities!![i].v
            var w = velocities!![i].w

            // Check for large velocities
            val translationx = v.x * h
            val translationy = v.y * h

            if (translationx * translationx + translationy * translationy > Settings.maxTranslationSquared) {
                val ratio =
                    Settings.maxTranslation / MathUtils.sqrt(translationx * translationx + translationy * translationy)
                v.x *= ratio
                v.y *= ratio
            }

            val rotation = h * w
            if (rotation * rotation > Settings.maxRotationSquared) {
                val ratio = Settings.maxRotation / MathUtils.abs(rotation)
                w *= ratio
            }

            // Integrate
            c.x += h * v.x
            c.y += h * v.y
            a += h * w

            positions!![i].a = a
            velocities!![i].w = w
        }

        // Solve position constraints
        timer.reset()
        var positionSolved = false
        for (i in 0 until step.positionIterations) {
            val contactsOkay = contactSolver.solvePositionConstraints()

            var jointsOkay = true
            for (j in 0 until jointCount) {
                val jointOkay = joints!![j].solvePositionConstraints(solverData)
                jointsOkay = jointsOkay && jointOkay
            }

            if (contactsOkay && jointsOkay) {
                // Exit early if the position errors are small.
                positionSolved = true
                break
            }
        }

        // Copy state buffers back to the bodies
        for (i in 0 until bodyCount) {
            val body = bodies!![i]
            body.sweep.c.x = positions!![i].c.x
            body.sweep.c.y = positions!![i].c.y
            body.sweep.a = positions!![i].a
            body._linearVelocity.x = velocities!![i].v.x
            body._linearVelocity.y = velocities!![i].v.y
            body._angularVelocity = velocities!![i].w
            body.synchronizeTransform()
        }

        profile.solvePosition.accum(timer.milliseconds)

        report(contactSolver.velocityConstraints)

        if (allowSleep) {
            var minSleepTime = Float.MAX_VALUE

            val linTolSqr = Settings.linearSleepTolerance * Settings.linearSleepTolerance
            val angTolSqr = Settings.angularSleepTolerance * Settings.angularSleepTolerance

            for (i in 0 until bodyCount) {
                val b = bodies!![i]
                if (b.type === BodyType.STATIC) {
                    continue
                }

                if (b.flags and Body.autoSleepFlag == 0
                    || b._angularVelocity * b._angularVelocity > angTolSqr
                    || Vec2.dot(b._linearVelocity, b._linearVelocity) > linTolSqr
                ) {
                    b.sleepTime = 0.0f
                    minSleepTime = 0.0f
                } else {
                    b.sleepTime += h
                    minSleepTime = MathUtils.min(minSleepTime, b.sleepTime)
                }
            }

            if (minSleepTime >= Settings.timeToSleep && positionSolved) {
                for (i in 0 until bodyCount) {
                    val b = bodies!![i]
                    b.isAwake = false
                }
            }
        }
    }

    fun solveTOI(subStep: TimeStep, toiIndexA: Int, toiIndexB: Int) {
        assert(toiIndexA < bodyCount)
        assert(toiIndexB < bodyCount)

        // Initialize the body state.
        for (i in 0 until bodyCount) {
            positions!![i].c.x = bodies!![i].sweep.c.x
            positions!![i].c.y = bodies!![i].sweep.c.y
            positions!![i].a = bodies!![i].sweep.a
            velocities!![i].v.x = bodies!![i]._linearVelocity.x
            velocities!![i].v.y = bodies!![i]._linearVelocity.y
            velocities!![i].w = bodies!![i]._angularVelocity
        }

        toiSolverDef.contacts = contacts
        toiSolverDef.count = contactCount
        toiSolverDef.step = subStep
        toiSolverDef.positions = positions
        toiSolverDef.velocities = velocities
        toiContactSolver.init(toiSolverDef)

        // Solve position constraints.
        for (i in 0 until subStep.positionIterations) {
            val contactsOkay = toiContactSolver.solveTOIPositionConstraints(toiIndexA, toiIndexB)
            if (contactsOkay) {
                break
            }
        }
        // #if 0
        // // Is the new position really safe?
        // for (int i = 0; i < m_contactCount; ++i)
        // {
        // Contact* c = m_contacts[i];
        // Fixture* fA = c.GetFixtureA();
        // Fixture* fB = c.GetFixtureB();
        //
        // Body bA = fA.GetBody();
        // Body bB = fB.GetBody();
        //
        // int indexA = c.GetChildIndexA();
        // int indexB = c.GetChildIndexB();
        //
        // DistanceInput input;
        // input.proxyA.Set(fA.GetShape(), indexA);
        // input.proxyB.Set(fB.GetShape(), indexB);
        // input.transformA = bA.GetTransform();
        // input.transformB = bB.GetTransform();
        // input.useRadii = false;
        //
        // DistanceOutput output;
        // SimplexCache cache;
        // cache.count = 0;
        // Distance(&output, &cache, &input);
        //
        // if (output.distance == 0 || cache.count == 3)
        // {
        // cache.count += 0;
        // }
        // }
        // #endif

        // Leap of faith to new safe state.
        bodies!![toiIndexA].sweep.c0.x = positions!![toiIndexA].c.x
        bodies!![toiIndexA].sweep.c0.y = positions!![toiIndexA].c.y
        bodies!![toiIndexA].sweep.a0 = positions!![toiIndexA].a
        bodies!![toiIndexB].sweep.c0.set(positions!![toiIndexB].c)
        bodies!![toiIndexB].sweep.a0 = positions!![toiIndexB].a

        // No warm starting is needed for TOI events because warm
        // starting impulses were applied in the discrete solver.
        toiContactSolver.initializeVelocityConstraints()

        // Solve velocity constraints.
        for (i in 0 until subStep.velocityIterations) {
            toiContactSolver.solveVelocityConstraints()
        }

        // Don't store the TOI contact forces for warm starting
        // because they can be quite large.

        val h = subStep.dt

        // Integrate positions
        for (i in 0 until bodyCount) {
            val c = positions!![i].c
            var a = positions!![i].a
            val v = velocities!![i].v
            var w = velocities!![i].w

            // Check for large velocities
            val translationx = v.x * h
            val translationy = v.y * h
            if (translationx * translationx + translationy * translationy > Settings.maxTranslationSquared) {
                val ratio =
                    Settings.maxTranslation / MathUtils.sqrt(translationx * translationx + translationy * translationy)
                v.mulLocal(ratio)
            }

            val rotation = h * w
            if (rotation * rotation > Settings.maxRotationSquared) {
                val ratio = Settings.maxRotation / MathUtils.abs(rotation)
                w *= ratio
            }

            // Integrate
            c.x += v.x * h
            c.y += v.y * h
            a += h * w

            positions!![i].c.x = c.x
            positions!![i].c.y = c.y
            positions!![i].a = a
            velocities!![i].v.x = v.x
            velocities!![i].v.y = v.y
            velocities!![i].w = w

            // Sync bodies
            val body = bodies!![i]
            body.sweep.c.x = c.x
            body.sweep.c.y = c.y
            body.sweep.a = a
            body._linearVelocity.x = v.x
            body._linearVelocity.y = v.y
            body._angularVelocity = w
            body.synchronizeTransform()
        }

        report(toiContactSolver.velocityConstraints)
    }

    fun add(body: Body) {
        assert(bodyCount < bodyCapacity)
        body.islandIndex = bodyCount
        bodies!![bodyCount] = body
        ++bodyCount
    }

    fun add(contact: Contact) {
        assert(contactCount < contactCapacity)
        contacts!![contactCount++] = contact
    }

    fun add(joint: Joint) {
        assert(jointCount < jointCapacity)
        joints!![jointCount++] = joint
    }

    fun report(constraints: Array<ContactVelocityConstraint>) {
        if (listener == null) return

        for (i in 0 until contactCount) {
            val c = contacts!![i]

            val vc = constraints[i]
            impulse.count = vc.pointCount
            for (j in 0 until vc.pointCount) {
                impulse.normalImpulses[j] = vc.points[j].normalImpulse
                impulse.tangentImpulses[j] = vc.points[j].tangentImpulse
            }

            listener!!.postSolve(c, impulse)
        }
    }
}
