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

import org.jbox2d.common.Mat22
import org.jbox2d.common.MathUtils
import org.jbox2d.common.Rot
import org.jbox2d.common.Settings
import org.jbox2d.common.Transform
import org.jbox2d.common.Vec2
import org.jbox2d.dynamics.SolverData
import org.jbox2d.internal.*
import org.jbox2d.pooling.IWorldPool

/**
 * A mouse joint is used to make a point on a body track a specified world point. This a soft
 * constraint with a maximum force. This allows the constraint to stretch and without applying huge
 * forces. NOTE: this joint is not documented in the manual because it was developed to be used in
 * the testbed. If you want to learn how to use the mouse joint, look at the testbed.
 *
 * @author Daniel
 */
class MouseJoint(argWorld: IWorldPool, def: MouseJointDef) : Joint(argWorld, def) {

    init {
        assert(def.target.isValid)
        assert(def.maxForce >= 0)
        assert(def.frequencyHz >= 0)
        assert(def.dampingRatio >= 0)
    }

    private val localAnchorB = Vec2()
    var target = Vec2()
        set(target) {
            if (!bodyB!!.isAwake) {
                bodyB!!.isAwake = true
            }
            this.target.set(target)
        }

    /** Frequency in Hertz. */
    var frequency: Float = def.frequencyHz

    /** Damping ratio (dimensionless) */
    var dampingRatio: Float = def.dampingRatio

    private var beta: Float = 0f

    // Solver shared
    private val impulse = Vec2()

    /* Maximum force in Newtons. */
    var maxForce: Float = def.maxForce

    private var gamma: Float = 0f

    // Solver temp
    private var indexB: Int = 0
    private val rB = Vec2()
    private val localCenterB = Vec2()
    private var invMassB: Float = 0f
    private var invIB: Float = 0f
    private val mass = Mat22()
    private val C = Vec2()

    init {
        target.set(def.target)
        Transform.mulTransToOutUnsafe(bodyB!!.xf, target, localAnchorB)
        impulse.setZero()
    }

    override fun getAnchorA(out: Vec2) {
        out.set(target)
    }

    override fun getAnchorB(out: Vec2) {
        bodyB!!.getWorldPointToOut(localAnchorB, out)
    }

    override fun getReactionForce(invDt: Float, out: Vec2) {
        out.set(impulse).mulLocal(invDt)
    }

    override fun getReactionTorque(invDt: Float): Float {
        return invDt * 0.0f
    }

    override fun initVelocityConstraints(data: SolverData) {
        indexB = bodyB!!.islandIndex
        localCenterB.set(bodyB!!.sweep.localCenter)
        invMassB = bodyB!!.invMass
        invIB = bodyB!!.invI

        val cB = data.positions!![indexB].c
        val aB = data.positions!![indexB].a
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w

        val qB = pool.popRot()

        qB.setRadians(aB)

        val mass = bodyB!!.mass

        // Frequency
        val omega = 2.0f * MathUtils.PI * frequency

        // Damping coefficient
        val d = 2.0f * mass * dampingRatio * omega

        // Spring stiffness
        val k = mass * (omega * omega)

        // magic formulas
        // gamma has units of inverse mass.
        // beta has units of inverse time.
        val h = data.step!!.dt
        assert(d + h * k > Settings.EPSILON)
        gamma = h * (d + h * k)
        if (gamma != 0.0f) {
            gamma = 1.0f / gamma
        }
        beta = h * k * gamma

        val temp = pool.popVec2()

        // Compute the effective mass matrix.
        Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(localCenterB), rB)

        // K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
        // = [1/m1+1/m2 0 ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
        // [ 0 1/m1+1/m2] [-r1.x*r1.y r1.x*r1.x] [-r1.x*r1.y r1.x*r1.x]
        val K = pool.popMat22()
        K.ex.x = invMassB + invIB * rB.y * rB.y + gamma
        K.ex.y = -invIB * rB.x * rB.y
        K.ey.x = K.ex.y
        K.ey.y = invMassB + invIB * rB.x * rB.x + gamma

        K.invertToOut(this.mass)

        C.set(cB).addLocal(rB).subLocal(target)
        C.mulLocal(beta)

        // Cheat with some damping
        wB *= 0.98f

        if (data.step!!.warmStarting) {
            impulse.mulLocal(data.step!!.dtRatio)
            vB.x += invMassB * impulse.x
            vB.y += invMassB * impulse.y
            wB += invIB * Vec2.cross(rB, impulse)
        } else {
            impulse.setZero()
        }

        data.velocities!![indexB].w = wB

        pool.pushVec2(1)
        pool.pushMat22(1)
        pool.pushRot(1)
    }

    override fun solvePositionConstraints(data: SolverData) = true

    override fun solveVelocityConstraints(data: SolverData) {
        val vB = data.velocities!![indexB].v
        var wB = data.velocities!![indexB].w

        // Cdot = v + cross(w, r)
        val Cdot = pool.popVec2()
        Vec2.crossToOutUnsafe(wB, rB, Cdot)
        Cdot.addLocal(vB)

        val impulse = pool.popVec2()
        val temp = pool.popVec2()

        temp.set(this.impulse).mulLocal(gamma).addLocal(C).addLocal(Cdot).negateLocal()
        Mat22.mulToOutUnsafe(mass, temp, impulse)

        val oldImpulse = temp
        oldImpulse.set(this.impulse)
        this.impulse.addLocal(impulse)
        val maxImpulse = data.step!!.dt * maxForce
        if (this.impulse.lengthSquared() > maxImpulse * maxImpulse) {
            this.impulse.mulLocal(maxImpulse / this.impulse.length())
        }
        impulse.set(this.impulse).subLocal(oldImpulse)

        vB.x += invMassB * impulse.x
        vB.y += invMassB * impulse.y
        wB += invIB * Vec2.cross(rB, impulse)

        data.velocities!![indexB].w = wB

        pool.pushVec2(3)
    }
}
