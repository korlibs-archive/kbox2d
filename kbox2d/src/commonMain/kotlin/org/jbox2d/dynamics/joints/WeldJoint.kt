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
/**
 * Created at 3:38:38 AM Jan 15, 2011
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
import org.jbox2d.pooling.IWorldPool

/**
 * A weld joint essentially glues two bodies together. A weld joint may distort somewhat because the
 * island constraint solver is approximate.
 *
 * @author Daniel Murphy
 */
class WeldJoint(argWorld: IWorldPool, def: WeldJointDef) : Joint(argWorld, def) {

    var frequency: Float = def.frequencyHz
    var dampingRatio: Float = def.dampingRatio
    private var bias: Float = 0f

    // Solver shared
    val localAnchorA: Vec2 = Vec2(def.localAnchorA)
    val localAnchorB: Vec2 = Vec2(def.localAnchorB)
    val referenceAngleRadians: Float = def.referenceAngleRadians
    val referenceAngleDegrees: Float get() = referenceAngleRadians * MathUtils.RAD2DEG
    val referenceAngle: Angle get() = referenceAngleRadians.radians
    private var gamma: Float = 0f
    private val impulse: Vec3 = Vec3(0f, 0f, 0f)

    // Solver temp
    private var indexA: Int = 0
    private var indexB: Int = 0
    private val rA = Vec2()
    private val rB = Vec2()
    private val localCenterA = Vec2()
    private val localCenterB = Vec2()
    private var invMassA: Float = 0.toFloat()
    private var invMassB: Float = 0.toFloat()
    private var invIA: Float = 0.toFloat()
    private var invIB: Float = 0.toFloat()
    private val mass = Mat33()

    override fun getAnchorA(out: Vec2) {
        bodyA!!.getWorldPointToOut(localAnchorA, out)
    }

    override fun getAnchorB(out: Vec2) {
        bodyB!!.getWorldPointToOut(localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.set(impulse.x, impulse.y)
        out.mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float {
        return invDt * impulse.z
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

        // Vec2 cA = data.positions[m_indexA].c;
        val aA = data.positions!![indexA].a
        val vA = data.velocities!![indexA].v
        var wA = data.velocities!![indexA].w

        // Vec2 cB = data.positions[m_indexB].c;
        val aB = data.positions!![indexB].a
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w

        val qA = pool.popRot()
        val qB = pool.popRot()
        val temp = pool.popVec2()

        qA.setRadians(aA)
        qB.setRadians(aB)

        // Compute the effective masses.
        Rot.mulToOutUnsafe(qA, temp.set(localAnchorA).subLocal(localCenterA), rA)
        Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(localCenterB), rB)

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

        val K = pool.popMat33()

        K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB
        K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB
        K.ez.x = -rA.y * iA - rB.y * iB
        K.ex.y = K.ey.x
        K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB
        K.ez.y = rA.x * iA + rB.x * iB
        K.ex.z = K.ez.x
        K.ey.z = K.ez.y
        K.ez.z = iA + iB

        if (frequency > 0.0f) {
            K.getInverse22(mass)

            var invM = iA + iB
            val m = if (invM > 0.0f) 1.0f / invM else 0.0f

            val C = aB - aA - referenceAngleRadians

            // Frequency
            val omega = 2.0f * MathUtils.PI * frequency

            // Damping coefficient
            val d = 2.0f * m * dampingRatio * omega

            // Spring stiffness
            val k = m * omega * omega

            // magic formulas
            val h = data.step!!.dt
            gamma = h * (d + h * k)
            gamma = if (gamma != 0.0f) 1.0f / gamma else 0.0f
            bias = C * h * k * gamma

            invM += gamma
            mass.ez.z = if (invM != 0.0f) 1.0f / invM else 0.0f
        } else {
            K.getSymInverse33(mass)
            gamma = 0.0f
            bias = 0.0f
        }

        if (data.step!!.warmStarting) {
            val P = pool.popVec2()
            // Scale impulses to support a variable time step.
            impulse.mulLocal(data.step!!.dtRatio)

            P.set(impulse.x, impulse.y)

            vA.x -= mA * P.x
            vA.y -= mA * P.y
            wA -= iA * (Vec2.cross(rA, P) + impulse.z)

            vB.x += mB * P.x
            vB.y += mB * P.y
            wB += iB * (Vec2.cross(rB, P) + impulse.z)
            pool.pushVec2(1)
        } else {
            impulse.setZero()
        }

        data.velocities!![indexA].w = wA
        data.velocities!![indexB].w = wB

        pool.pushVec2(1)
        pool.pushRot(2)
        pool.pushMat33(1)
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

        val Cdot1 = pool.popVec2()
        val P = pool.popVec2()
        val temp = pool.popVec2()
        if (frequency > 0.0f) {
            val Cdot2 = wB - wA

            val impulse2 = -mass.ez.z * (Cdot2 + bias + gamma * impulse.z)
            impulse.z += impulse2

            wA -= iA * impulse2
            wB += iB * impulse2

            Vec2.crossToOutUnsafe(wB, rB, Cdot1)
            Vec2.crossToOutUnsafe(wA, rA, temp)
            Cdot1.addLocal(vB).subLocal(vA).subLocal(temp)

            val impulse1 = P
            Mat33.mul22ToOutUnsafe(mass, Cdot1, impulse1)
            impulse1.negateLocal()

            impulse.x += impulse1.x
            impulse.y += impulse1.y

            vA.x -= mA * P.x
            vA.y -= mA * P.y
            wA -= iA * Vec2.cross(rA, P)

            vB.x += mB * P.x
            vB.y += mB * P.y
            wB += iB * Vec2.cross(rB, P)
        } else {
            Vec2.crossToOutUnsafe(wA, rA, temp)
            Vec2.crossToOutUnsafe(wB, rB, Cdot1)
            Cdot1.addLocal(vB).subLocal(vA).subLocal(temp)
            val Cdot2 = wB - wA

            val Cdot = pool.popVec3()
            Cdot.set(Cdot1.x, Cdot1.y, Cdot2)

            val impulse = pool.popVec3()
            Mat33.mulToOutUnsafe(mass, Cdot, impulse)
            impulse.negateLocal()
            this.impulse.addLocal(impulse)

            P.set(impulse.x, impulse.y)

            vA.x -= mA * P.x
            vA.y -= mA * P.y
            wA -= iA * (Vec2.cross(rA, P) + impulse.z)

            vB.x += mB * P.x
            vB.y += mB * P.y
            wB += iB * (Vec2.cross(rB, P) + impulse.z)

            pool.pushVec3(2)
        }

        data.velocities!![indexA].w = wA
        data.velocities!![indexB].w = wB

        pool.pushVec2(3)
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        val cA = data.positions!![indexA].c
        var aA = data.positions!![indexA].a
        val cB = data.positions!![indexB].c
        var aB = data.positions!![indexB].a
        val qA = pool.popRot()
        val qB = pool.popRot()
        val temp = pool.popVec2()
        val rA = pool.popVec2()
        val rB = pool.popVec2()

        qA.setRadians(aA)
        qB.setRadians(aB)

        val mA = invMassA
        val mB = invMassB
        val iA = invIA
        val iB = invIB

        Rot.mulToOutUnsafe(qA, temp.set(localAnchorA).subLocal(localCenterA), rA)
        Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(localCenterB), rB)
        val positionError: Float
        val angularError: Float

        val K = pool.popMat33()
        val C1 = pool.popVec2()
        val P = pool.popVec2()

        K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB
        K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB
        K.ez.x = -rA.y * iA - rB.y * iB
        K.ex.y = K.ey.x
        K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB
        K.ez.y = rA.x * iA + rB.x * iB
        K.ex.z = K.ez.x
        K.ey.z = K.ez.y
        K.ez.z = iA + iB
        if (frequency > 0.0f) {
            C1.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)

            positionError = C1.length()
            angularError = 0.0f

            K.solve22ToOut(C1, P)
            P.negateLocal()

            cA.x -= mA * P.x
            cA.y -= mA * P.y
            aA -= iA * Vec2.cross(rA, P)

            cB.x += mB * P.x
            cB.y += mB * P.y
            aB += iB * Vec2.cross(rB, P)
        } else {
            C1.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)
            val C2 = aB - aA - referenceAngleRadians

            positionError = C1.length()
            angularError = MathUtils.abs(C2)

            val C = pool.popVec3()
            val impulse = pool.popVec3()
            C.set(C1.x, C1.y, C2)

            K.solve33ToOut(C, impulse)
            impulse.negateLocal()
            P.set(impulse.x, impulse.y)

            cA.x -= mA * P.x
            cA.y -= mA * P.y
            aA -= iA * (Vec2.cross(rA, P) + impulse.z)

            cB.x += mB * P.x
            cB.y += mB * P.y
            aB += iB * (Vec2.cross(rB, P) + impulse.z)
            pool.pushVec3(2)
        }

        data.positions!![indexA].a = aA
        data.positions!![indexB].a = aB

        pool.pushVec2(5)
        pool.pushRot(2)
        pool.pushMat33(1)

        return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop
    }
}
