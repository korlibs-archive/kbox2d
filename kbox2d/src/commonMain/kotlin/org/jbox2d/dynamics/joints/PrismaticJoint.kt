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
package org.jbox2d.dynamics.joints

import com.soywiz.korma.geom.*
import org.jbox2d.common.Mat33
import org.jbox2d.common.MathUtils
import org.jbox2d.common.Rot
import org.jbox2d.common.Settings
import org.jbox2d.common.Vec2
import org.jbox2d.common.Vec3
import org.jbox2d.dynamics.SolverData
import org.jbox2d.internal.*
import org.jbox2d.pooling.IWorldPool

/**
 * A prismatic joint. This joint provides one degree of freedom: translation along an axis fixed in
 * bodyA. Relative rotation is prevented. You can use a joint limit to restrict the range of motion
 * and a joint motor to drive the motion or to model joint friction.
 *
 * @author Daniel
 */
class PrismaticJoint(argWorld: IWorldPool, def: PrismaticJointDef) : Joint(argWorld, def) {

    // Solver shared

    val localAnchorA: Vec2 = Vec2(def.localAnchorA)
    val localAnchorB: Vec2 = Vec2(def.localAnchorB)

    val localXAxisA: Vec2 = Vec2(def.localAxisA)
    protected val localYAxisA: Vec2 = Vec2()

    var referenceAngleRadians: Float = 0f
        protected set
    var referenceAngleDegrees: Float
        get() = referenceAngleRadians * MathUtils.RAD2DEG
        protected set(value) = run { referenceAngleRadians = value * MathUtils.DEG2RAD }
    var referenceAngle: Angle
        get() = referenceAngleRadians.radians
        protected set(value) = run { referenceAngleRadians = value.radians.toFloat() }

    private val impulse: Vec3 = Vec3()
    private var motorImpulse: Float = 0f

    /**
     * Lower joint limit, usually in meters.
     */
    var lowerLimit: Float = def.lowerTranslation
        private set

    /**
     * Upper joint limit, usually in meters.
     */
    var upperLimit: Float = def.upperTranslation
        private set

    private var _maxMotorForce: Float = def.maxMotorForce
    private var _motorSpeed: Float = def.motorSpeed

    /**
     * Is the joint limit enabled?
     */
    var isLimitEnabled: Boolean = def.enableLimit
        private set

    /**
     * Is the joint motor enabled?
     */
    var isMotorEnabled: Boolean = def.enableMotor
        private set

    private var limitState: LimitState = LimitState.INACTIVE

    // Solver temp
    private var indexA: Int = 0
    private var indexB: Int = 0
    private val localCenterA = Vec2()
    private val localCenterB = Vec2()
    private var invMassA: Float = 0f
    private var invMassB: Float = 0f
    private var invIA: Float = 0f
    private var invIB: Float = 0f
    private val axis: Vec2 = Vec2()
    private val perp: Vec2 = Vec2()
    private var s1: Float = 0f
    private var s2: Float = 0f
    private var a1: Float = 0f
    private var a2: Float = 0f
    private val K: Mat33 = Mat33()
    private var motorMass: Float = 0f // effective mass for motor/limit translational constraint.

    /**
     * Current joint translation, usually in meters.
     */
    val jointSpeed: Float
        get() {
            val bA = bodyA
            val bB = bodyB

            val temp = pool.popVec2()
            val rA = pool.popVec2()
            val rB = pool.popVec2()
            val p1 = pool.popVec2()
            val p2 = pool.popVec2()
            val d = pool.popVec2()
            val axis = pool.popVec2()
            val temp2 = pool.popVec2()
            val temp3 = pool.popVec2()

            temp.set(localAnchorA).subLocal(bA!!.sweep.localCenter)
            Rot.mulToOutUnsafe(bA.xf.q, temp, rA)

            temp.set(localAnchorB).subLocal(bB!!.sweep.localCenter)
            Rot.mulToOutUnsafe(bB.xf.q, temp, rB)

            p1.set(bA.sweep.c).addLocal(rA)
            p2.set(bB.sweep.c).addLocal(rB)

            d.set(p2).subLocal(p1)
            Rot.mulToOutUnsafe(bA.xf.q, localXAxisA, axis)

            val vA = bA._linearVelocity
            val vB = bB._linearVelocity
            val wA = bA._angularVelocity
            val wB = bB._angularVelocity


            Vec2.crossToOutUnsafe(wA, axis, temp)
            Vec2.crossToOutUnsafe(wB, rB, temp2)
            Vec2.crossToOutUnsafe(wA, rA, temp3)

            temp2.addLocal(vB).subLocal(vA).subLocal(temp3)
            val speed = Vec2.dot(d, temp) + Vec2.dot(axis, temp2)

            pool.pushVec2(9)

            return speed
        }

    val jointTranslation: Float
        get() {
            val pA = pool.popVec2()
            val pB = pool.popVec2()
            val axis = pool.popVec2()
            bodyA!!.getWorldPointToOut(localAnchorA, pA)
            bodyB!!.getWorldPointToOut(localAnchorB, pB)
            bodyA!!.getWorldVectorToOutUnsafe(localXAxisA, axis)
            pB.subLocal(pA)
            val translation = Vec2.dot(pB, axis)
            pool.pushVec2(3)
            return translation
        }

    /**
     * Motor speed, usually in meters per second.
     */
    var motorSpeed: Float
        get() = _motorSpeed
        set(speed) {
            bodyA!!.isAwake = true
            bodyB!!.isAwake = true
            _motorSpeed = speed
        }

    /**
     * Maximum motor force, usually in N.
     */
    var maxMotorForce: Float
        get() = _maxMotorForce
        set(force) {
            bodyA!!.isAwake = true
            bodyB!!.isAwake = true
            _maxMotorForce = force
        }

    init {
        localXAxisA.normalize()
        Vec2.crossToOutUnsafe(1f, localXAxisA, localYAxisA)
        referenceAngleRadians = def.referenceAngleRadians
    }

    override fun getAnchorA(out: Vec2) {
        bodyA!!.getWorldPointToOut(localAnchorA, out)
    }

    override fun getAnchorB(out: Vec2) {
        bodyB!!.getWorldPointToOut(localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        val temp = pool.popVec2()
        temp.set(axis).mulLocal(motorImpulse + impulse.z)
        out.set(perp).mulLocal(impulse.x).addLocal(temp).mulLocal(invDt)
        pool.pushVec2(1)
    }

    override fun getReactionTorque(invDt: Float): Float {
        return invDt * impulse.y
    }

    /**
     * Enable/disable the joint limit.
     */
    fun enableLimit(flag: Boolean) {
        if (flag != isLimitEnabled) {
            bodyA!!.isAwake = true
            bodyB!!.isAwake = true
            isLimitEnabled = flag
            impulse.z = 0.0f
        }
    }

    /**
     * Set the joint limits, usually in meters.
     */
    fun setLimits(lower: Float, upper: Float) {
        assert(lower <= upper)
        if (lower != lowerLimit || upper != upperLimit) {
            bodyA!!.isAwake = true
            bodyB!!.isAwake = true
            lowerLimit = lower
            upperLimit = upper
            impulse.z = 0.0f
        }
    }

    /**
     * Enable/disable the joint motor.
     */
    fun enableMotor(flag: Boolean) {
        bodyA!!.isAwake = true
        bodyB!!.isAwake = true
        isMotorEnabled = flag
    }

    /**
     * Get the current motor force, usually in N.
     */
    fun getMotorForce(invDt: Float): Float {
        return motorImpulse * invDt
    }

    override fun initVelocityConstraints(data: SolverData) {
        indexA = bodyA!!.islandIndex
        indexB = bodyB!!.islandIndex
        localCenterA.set(bodyA!!.sweep.localCenter)
        localCenterB.set(bodyB!!.sweep.localCenter)
        invMassA = bodyA!!.invMass
        invMassB = bodyB!!.invMass
        invIA = bodyA!!.invI
        invIB = bodyB!!.invI

        val cA = data.positions!![indexA].c
        val aA = data.positions!![indexA].a
        val vA = data.velocities!![indexA].v
        var wA = data.velocities!![indexA].w

        val cB = data.positions!![indexB].c
        val aB = data.positions!![indexB].a
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w

        val qA = pool.popRot()
        val qB = pool.popRot()
        val d = pool.popVec2()
        val temp = pool.popVec2()
        val rA = pool.popVec2()
        val rB = pool.popVec2()

        qA.setRadians(aA)
        qB.setRadians(aB)

        // Compute the effective masses.
        Rot.mulToOutUnsafe(qA, d.set(localAnchorA).subLocal(localCenterA), rA)
        Rot.mulToOutUnsafe(qB, d.set(localAnchorB).subLocal(localCenterB), rB)
        d.set(cB).subLocal(cA).addLocal(rB).subLocal(rA)

        val mA = invMassA
        val mB = invMassB
        val iA = invIA
        val iB = invIB

        // Compute motor Jacobian and effective mass.
        run {
            Rot.mulToOutUnsafe(qA, localXAxisA, axis)
            temp.set(d).addLocal(rA)
            a1 = Vec2.cross(temp, axis)
            a2 = Vec2.cross(rB, axis)

            motorMass = mA + mB + iA * a1 * a1 + iB * a2 * a2
            if (motorMass > 0.0f) {
                motorMass = 1.0f / motorMass
            }
        }

        // Prismatic constraint.
        run {
            Rot.mulToOutUnsafe(qA, localYAxisA, perp)

            temp.set(d).addLocal(rA)
            s1 = Vec2.cross(temp, perp)
            s2 = Vec2.cross(rB, perp)

            val k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2
            val k12 = iA * s1 + iB * s2
            val k13 = iA * s1 * a1 + iB * s2 * a2
            var k22 = iA + iB
            if (k22 == 0.0f) {
                // For bodies with fixed rotation.
                k22 = 1.0f
            }
            val k23 = iA * a1 + iB * a2
            val k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2

            K.ex.set(k11, k12, k13)
            K.ey.set(k12, k22, k23)
            K.ez.set(k13, k23, k33)
        }

        // Compute motor and limit terms.
        if (isLimitEnabled) {
            val jointTranslation = Vec2.dot(axis, d)
            if (MathUtils.abs(upperLimit - lowerLimit) < 2.0f * Settings.linearSlop) {
                limitState = LimitState.EQUAL
            } else if (jointTranslation <= lowerLimit) {
                if (limitState !== LimitState.AT_LOWER) {
                    limitState = LimitState.AT_LOWER
                    impulse.z = 0.0f
                }
            } else if (jointTranslation >= upperLimit) {
                if (limitState !== LimitState.AT_UPPER) {
                    limitState = LimitState.AT_UPPER
                    impulse.z = 0.0f
                }
            } else {
                limitState = LimitState.INACTIVE
                impulse.z = 0.0f
            }
        } else {
            limitState = LimitState.INACTIVE
            impulse.z = 0.0f
        }

        if (!isMotorEnabled) {
            motorImpulse = 0.0f
        }

        if (data.step!!.warmStarting) {
            // Account for variable time step.
            impulse.mulLocal(data.step!!.dtRatio)
            motorImpulse *= data.step!!.dtRatio

            val P = pool.popVec2()
            temp.set(axis).mulLocal(motorImpulse + impulse.z)
            P.set(perp).mulLocal(impulse.x).addLocal(temp)

            val LA = impulse.x * s1 + impulse.y + (motorImpulse + impulse.z) * a1
            val LB = impulse.x * s2 + impulse.y + (motorImpulse + impulse.z) * a2

            vA.x -= mA * P.x
            vA.y -= mA * P.y
            wA -= iA * LA

            vB.x += mB * P.x
            vB.y += mB * P.y
            wB += iB * LB

            pool.pushVec2(1)
        } else {
            impulse.setZero()
            motorImpulse = 0.0f
        }

        // data.velocities[m_indexA].v.set(vA);
        data.velocities!![indexA].w = wA
        // data.velocities[m_indexB].v.set(vB);
        data.velocities!![indexB].w = wB

        pool.pushRot(2)
        pool.pushVec2(4)
    }

    override fun solveVelocityConstraints(data: SolverData) {
        val vA = data.velocities!![indexA].v
        var wA = data.velocities!![indexA].w
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w

        val mA = invMassA
        val mB = invMassB
        val iA = invIA
        val iB = invIB

        val temp = pool.popVec2()

        // Solve linear motor constraint.
        if (isMotorEnabled && limitState !== LimitState.EQUAL) {
            temp.set(vB).subLocal(vA)
            val Cdot = Vec2.dot(axis, temp) + a2 * wB - a1 * wA
            var impulse = motorMass * (_motorSpeed - Cdot)
            val oldImpulse = motorImpulse
            val maxImpulse = data.step!!.dt * _maxMotorForce
            motorImpulse = MathUtils.clamp(motorImpulse + impulse, -maxImpulse, maxImpulse)
            impulse = motorImpulse - oldImpulse

            val P = pool.popVec2()
            P.set(axis).mulLocal(impulse)
            val LA = impulse * a1
            val LB = impulse * a2

            vA.x -= mA * P.x
            vA.y -= mA * P.y
            wA -= iA * LA

            vB.x += mB * P.x
            vB.y += mB * P.y
            wB += iB * LB

            pool.pushVec2(1)
        }

        val Cdot1 = pool.popVec2()
        temp.set(vB).subLocal(vA)
        Cdot1.x = Vec2.dot(perp, temp) + s2 * wB - s1 * wA
        Cdot1.y = wB - wA

        if (isLimitEnabled && limitState !== LimitState.INACTIVE) {
            // Solve prismatic and limit constraint in block form.
            temp.set(vB).subLocal(vA)
            val Cdot2 = Vec2.dot(axis, temp) + a2 * wB - a1 * wA

            val Cdot = pool.popVec3()
            Cdot.set(Cdot1.x, Cdot1.y, Cdot2)

            val f1 = pool.popVec3()
            val df = pool.popVec3()

            f1.set(impulse)
            K.solve33ToOut(Cdot.negateLocal(), df)
            // Cdot.negateLocal(); not used anymore
            impulse.addLocal(df)

            if (limitState === LimitState.AT_LOWER) {
                impulse.z = MathUtils.max(impulse.z, 0.0f)
            } else if (limitState === LimitState.AT_UPPER) {
                impulse.z = MathUtils.min(impulse.z, 0.0f)
            }

            // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) +
            // f1(1:2)
            val b = pool.popVec2()
            val f2r = pool.popVec2()

            temp.set(K.ez.x, K.ez.y).mulLocal(impulse.z - f1.z)
            b.set(Cdot1).negateLocal().subLocal(temp)

            K.solve22ToOut(b, f2r)
            f2r.addLocal(f1.x, f1.y)
            impulse.x = f2r.x
            impulse.y = f2r.y

            df.set(impulse).subLocal(f1)

            val P = pool.popVec2()
            temp.set(axis).mulLocal(df.z)
            P.set(perp).mulLocal(df.x).addLocal(temp)

            val LA = df.x * s1 + df.y + df.z * a1
            val LB = df.x * s2 + df.y + df.z * a2

            vA.x -= mA * P.x
            vA.y -= mA * P.y
            wA -= iA * LA

            vB.x += mB * P.x
            vB.y += mB * P.y
            wB += iB * LB

            pool.pushVec2(3)
            pool.pushVec3(3)
        } else {
            // Limit is inactive, just solve the prismatic constraint in block form.
            val df = pool.popVec2()
            K.solve22ToOut(Cdot1.negateLocal(), df)
            Cdot1.negateLocal()

            impulse.x += df.x
            impulse.y += df.y

            val P = pool.popVec2()
            P.set(perp).mulLocal(df.x)
            val LA = df.x * s1 + df.y
            val LB = df.x * s2 + df.y

            vA.x -= mA * P.x
            vA.y -= mA * P.y
            wA -= iA * LA

            vB.x += mB * P.x
            vB.y += mB * P.y
            wB += iB * LB

            pool.pushVec2(2)
        }

        data.velocities!![indexA].w = wA
        data.velocities!![indexB].w = wB

        pool.pushVec2(2)
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        val qA = pool.popRot()
        val qB = pool.popRot()
        val rA = pool.popVec2()
        val rB = pool.popVec2()
        val d = pool.popVec2()
        val axis = pool.popVec2()
        val perp = pool.popVec2()
        val temp = pool.popVec2()
        val C1 = pool.popVec2()

        val impulse = pool.popVec3()

        val cA = data.positions!![indexA].c
        var aA = data.positions!![indexA].a
        val cB = data.positions!![indexB].c
        var aB = data.positions!![indexB].a

        qA.setRadians(aA)
        qB.setRadians(aB)

        val mA = invMassA
        val mB = invMassB
        val iA = invIA
        val iB = invIB

        // Compute fresh Jacobians
        Rot.mulToOutUnsafe(qA, temp.set(localAnchorA).subLocal(localCenterA), rA)
        Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(localCenterB), rB)
        d.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)

        Rot.mulToOutUnsafe(qA, localXAxisA, axis)
        val a1 = Vec2.cross(temp.set(d).addLocal(rA), axis)
        val a2 = Vec2.cross(rB, axis)
        Rot.mulToOutUnsafe(qA, localYAxisA, perp)

        val s1 = Vec2.cross(temp.set(d).addLocal(rA), perp)
        val s2 = Vec2.cross(rB, perp)

        C1.x = Vec2.dot(perp, d)
        C1.y = aB - aA - referenceAngleRadians

        var linearError = MathUtils.abs(C1.x)
        val angularError = MathUtils.abs(C1.y)

        var active = false
        var C2 = 0.0f
        if (isLimitEnabled) {
            val translation = Vec2.dot(axis, d)
            if (MathUtils.abs(upperLimit - lowerLimit) < 2.0f * Settings.linearSlop) {
                // Prevent large angular corrections
                C2 = MathUtils.clamp(
                    translation, -Settings.maxLinearCorrection,
                    Settings.maxLinearCorrection
                )
                linearError = MathUtils.max(linearError, MathUtils.abs(translation))
                active = true
            } else if (translation <= lowerLimit) {
                // Prevent large linear corrections and allow some slop.
                C2 = MathUtils.clamp(
                    translation - lowerLimit + Settings.linearSlop,
                    -Settings.maxLinearCorrection, 0.0f
                )
                linearError = MathUtils.max(linearError, lowerLimit - translation)
                active = true
            } else if (translation >= upperLimit) {
                // Prevent large linear corrections and allow some slop.
                C2 = MathUtils.clamp(
                    translation - upperLimit - Settings.linearSlop, 0.0f,
                    Settings.maxLinearCorrection
                )
                linearError = MathUtils.max(linearError, translation - upperLimit)
                active = true
            }
        }

        if (active) {
            val k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2
            val k12 = iA * s1 + iB * s2
            val k13 = iA * s1 * a1 + iB * s2 * a2
            var k22 = iA + iB
            if (k22 == 0.0f) {
                // For fixed rotation
                k22 = 1.0f
            }
            val k23 = iA * a1 + iB * a2
            val k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2

            val K = pool.popMat33()
            K.ex.set(k11, k12, k13)
            K.ey.set(k12, k22, k23)
            K.ez.set(k13, k23, k33)

            val C = pool.popVec3()
            C.x = C1.x
            C.y = C1.y
            C.z = C2

            K.solve33ToOut(C.negateLocal(), impulse)
            pool.pushVec3(1)
            pool.pushMat33(1)
        } else {
            val k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2
            val k12 = iA * s1 + iB * s2
            var k22 = iA + iB
            if (k22 == 0.0f) {
                k22 = 1.0f
            }

            val K = pool.popMat22()
            K.ex.set(k11, k12)
            K.ey.set(k12, k22)

            // temp is impulse1
            K.solveToOut(C1.negateLocal(), temp)
            C1.negateLocal()

            impulse.x = temp.x
            impulse.y = temp.y
            impulse.z = 0.0f

            pool.pushMat22(1)
        }

        val Px = impulse.x * perp.x + impulse.z * axis.x
        val Py = impulse.x * perp.y + impulse.z * axis.y
        val LA = impulse.x * s1 + impulse.y + impulse.z * a1
        val LB = impulse.x * s2 + impulse.y + impulse.z * a2

        cA.x -= mA * Px
        cA.y -= mA * Py
        aA -= iA * LA
        cB.x += mB * Px
        cB.y += mB * Py
        aB += iB * LB

        data.positions!![indexA].a = aA
        data.positions!![indexB].a = aB

        pool.pushVec2(7)
        pool.pushVec3(1)
        pool.pushRot(2)

        return linearError <= Settings.linearSlop && angularError <= Settings.angularSlop
    }
}
