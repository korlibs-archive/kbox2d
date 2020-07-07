package org.jbox2d.dynamics.joints

import org.jbox2d.common.Mat22
import org.jbox2d.common.MathUtils
import org.jbox2d.common.Rot
import org.jbox2d.common.Vec2
import org.jbox2d.dynamics.SolverData
import org.jbox2d.internal.*
import org.jbox2d.pooling.IWorldPool

/**
 * A motor joint is used to control the relative motion between two bodies. A typical usage is to
 * control the movement of a dynamic body with respect to the ground.
 *
 * @author dmurph
 */
class MotorJoint(pool: IWorldPool, def: MotorJointDef) : Joint(pool, def) {

    // Solver shared
    /** The target linear offset, in frame A, in meters. Do not modify. */
    var linearOffset = Vec2()
        set(linearOffset) {
            if (linearOffset.x != this.linearOffset.x || linearOffset.y != this.linearOffset.y) {
                bodyA!!.isAwake = true
                bodyB!!.isAwake = true
                this.linearOffset.set(linearOffset)
            }
        }
    private var _angularOffset: Float = def.angularOffset
    private val linearImpulse = Vec2()
    private var angularImpulse: Float = 0f
    private var _maxForce: Float = def.maxForce
    private var _maxTorque: Float = def.maxTorque

    var correctionFactor: Float = def.correctionFactor

    // Solver temp
    private var indexA: Int = 0
    private var indexB: Int = 0
    private val rA = Vec2()
    private val rB = Vec2()
    private val localCenterA = Vec2()
    private val localCenterB = Vec2()
    private val linearError = Vec2()
    private var angularError: Float = 0f
    private var invMassA: Float = 0f
    private var invMassB: Float = 0f
    private var invIA: Float = 0f
    private var invIB: Float = 0f
    private val linearMass = Mat22()
    private var angularMass: Float = 0f

    /**
     * Target angular offset in radians.
     */
    var angularOffset: Float
        get() = _angularOffset
        set(angularOffset) {
            if (angularOffset != _angularOffset) {
                bodyA!!.isAwake = true
                bodyB!!.isAwake = true
                _angularOffset = angularOffset
            }
        }

    /**
     * Maximum friction force in N.
     */
    var maxForce: Float
        get() = _maxForce
        set(force) {
            assert(force >= 0.0f)
            _maxForce = force
        }

    /**
     * Maximum friction torque in N*m.
     */
    var maxTorque: Float
        get() = _maxTorque
        set(torque) {
            assert(torque >= 0.0f)
            _maxTorque = torque
        }

    init {
        linearOffset.set(def.linearOffset)
    }

    override fun getAnchorA(out: Vec2) {
        out.set(bodyA!!.position)
    }

    override fun getAnchorB(out: Vec2) {
        out.set(bodyB!!.position)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.set(linearImpulse).mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float {
        return angularImpulse * invDt
    }

    /**
     * Get the target linear offset, in frame A, in meters.
     */
    fun getLinearOffset(out: Vec2) {
        out.set(linearOffset)
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
        val temp = pool.popVec2()
        val K = pool.popMat22()

        qA.setRadians(aA)
        qB.setRadians(aB)

        // Compute the effective mass matrix.
        // m_rA = b2Mul(qA, -m_localCenterA);
        // m_rB = b2Mul(qB, -m_localCenterB);
        rA.x = qA.c * -localCenterA.x - qA.s * -localCenterA.y
        rA.y = qA.s * -localCenterA.x + qA.c * -localCenterA.y
        rB.x = qB.c * -localCenterB.x - qB.s * -localCenterB.y
        rB.y = qB.s * -localCenterB.x + qB.c * -localCenterB.y

        // J = [-I -r1_skew I r2_skew]
        // [ 0 -1 0 1]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
        // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
        // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]
        val mA = invMassA
        val mB = invMassB
        val iA = invIA
        val iB = invIB

        K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y
        K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y
        K.ey.x = K.ex.y
        K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x

        K.invertToOut(linearMass)

        angularMass = iA + iB
        if (angularMass > 0.0f) {
            angularMass = 1.0f / angularMass
        }

        // linearError = cB + m_rB - cA - m_rA - b2Mul(qA, m_linearOffset);
        Rot.mulToOutUnsafe(qA, linearOffset, temp)
        linearError.x = cB.x + rB.x - cA.x - rA.x - temp.x
        linearError.y = cB.y + rB.y - cA.y - rA.y - temp.y
        angularError = aB - aA - _angularOffset

        if (data.step!!.warmStarting) {
            // Scale impulses to support a variable time step.
            linearImpulse.x *= data.step!!.dtRatio
            linearImpulse.y *= data.step!!.dtRatio
            angularImpulse *= data.step!!.dtRatio

            val P = linearImpulse
            vA.x -= mA * P.x
            vA.y -= mA * P.y
            wA -= iA * (rA.x * P.y - rA.y * P.x + angularImpulse)
            vB.x += mB * P.x
            vB.y += mB * P.y
            wB += iB * (rB.x * P.y - rB.y * P.x + angularImpulse)
        } else {
            linearImpulse.setZero()
            angularImpulse = 0.0f
        }

        pool.pushVec2(1)
        pool.pushMat22(1)
        pool.pushRot(2)

        // data.velocities[m_indexA].v = vA;
        data.velocities!![indexA].w = wA
        // data.velocities[m_indexB].v = vB;
        data.velocities!![indexB].w = wB
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

        val h = data.step!!.dt
        val inv_h = data.step!!.invDt

        val temp = pool.popVec2()

        // Solve angular friction
        run {
            val Cdot = wB - wA + inv_h * correctionFactor * angularError
            var impulse = -angularMass * Cdot

            val oldImpulse = angularImpulse
            val maxImpulse = h * _maxTorque
            angularImpulse = MathUtils.clamp(angularImpulse + impulse, -maxImpulse, maxImpulse)
            impulse = angularImpulse - oldImpulse

            wA -= iA * impulse
            wB += iB * impulse
        }

        val Cdot = pool.popVec2()

        // Solve linear friction
        run {
            // Cdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA) + inv_h * m_correctionFactor *
            // linearError;
            Cdot.x = vB.x + -wB * rB.y - vA.x - -wA * rA.y + inv_h * correctionFactor * linearError.x
            Cdot.y = vB.y + wB * rB.x - vA.y - wA * rA.x + inv_h * correctionFactor * linearError.y

            val impulse = temp
            Mat22.mulToOutUnsafe(linearMass, Cdot, impulse)
            impulse.negateLocal()
            val oldImpulse = pool.popVec2()
            oldImpulse.set(linearImpulse)
            linearImpulse.addLocal(impulse)

            val maxImpulse = h * _maxForce

            if (linearImpulse.lengthSquared() > maxImpulse * maxImpulse) {
                linearImpulse.normalize()
                linearImpulse.mulLocal(maxImpulse)
            }

            impulse.x = linearImpulse.x - oldImpulse.x
            impulse.y = linearImpulse.y - oldImpulse.y

            vA.x -= mA * impulse.x
            vA.y -= mA * impulse.y
            wA -= iA * (rA.x * impulse.y - rA.y * impulse.x)

            vB.x += mB * impulse.x
            vB.y += mB * impulse.y
            wB += iB * (rB.x * impulse.y - rB.y * impulse.x)
        }

        pool.pushVec2(3)

        // data.velocities[m_indexA].v.set(vA);
        data.velocities!![indexA].w = wA
        // data.velocities[m_indexB].v.set(vB);
        data.velocities!![indexB].w = wB
    }

    override fun solvePositionConstraints(data: SolverData) = true
}
