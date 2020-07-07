package org.jbox2d.particle

import com.soywiz.korma.geom.*
import org.jbox2d.common.Transform
import org.jbox2d.common.Vec2

class ParticleGroup {

    internal var system: ParticleSystem? = null

    var firstIndex: Int = 0
    internal var lastIndex: Int = 0

    var groupFlags: Int = 0
    
    internal var strength: Float = 1.0f

    internal var prev: ParticleGroup? = null
    var next: ParticleGroup? = null

    internal var timestamp: Int = -1
    internal var _mass: Float = 0f
    internal var _inertia: Float = 0f

    internal val _center = Vec2()

    internal val _linearVelocity = Vec2()
    internal var _angularVelocity: Float = 0f

    val transform = Transform().apply { setIdentity() }

    internal var destroyAutomatically: Boolean = true
    internal var toBeDestroyed: Boolean = false
    internal var toBeSplit: Boolean = false

    var userData: Any? = null

    val particleCount: Int
        get() = lastIndex - firstIndex

    val mass: Float
        get() {
            updateStatistics()
            return _mass
        }

    val inertia: Float
        get() {
            updateStatistics()
            return _inertia
        }

    val center: Vec2
        get() {
            updateStatistics()
            return _center
        }

    val linearVelocity: Vec2
        get() {
            updateStatistics()
            return _linearVelocity
        }

    val angularVelocity: Float
        get() {
            updateStatistics()
            return _angularVelocity
        }

    val position: Vec2
        get() = transform.p

    val angleRadians: Float get() = transform.q.angleRadians
    val angleDegrees: Float get() = transform.q.angleDegrees
    val angle: Angle get() = transform.q.angle

    fun updateStatistics() {
        if (timestamp != system!!.timestamp) {
            val m = system!!.particleMass
            _mass = 0f
            _center.setZero()
            _linearVelocity.setZero()
            for (i in firstIndex until lastIndex) {
                _mass += m
                val pos = system!!.positionBuffer.data!![i]
                _center.x += m * pos.x
                _center.y += m * pos.y
                val vel = system!!.velocityBuffer.data!![i]
                _linearVelocity.x += m * vel.x
                _linearVelocity.y += m * vel.y
            }
            if (_mass > 0) {
                _center.x *= 1 / _mass
                _center.y *= 1 / _mass
                _linearVelocity.x *= 1 / _mass
                _linearVelocity.y *= 1 / _mass
            }
            _inertia = 0f
            _angularVelocity = 0f
            for (i in firstIndex until lastIndex) {
                val pos = system!!.positionBuffer.data!![i]
                val vel = system!!.velocityBuffer.data!![i]
                val px = pos.x - _center.x
                val py = pos.y - _center.y
                val vx = vel.x - _linearVelocity.x
                val vy = vel.y - _linearVelocity.y
                _inertia += m * (px * px + py * py)
                _angularVelocity += m * (px * vy - py * vx)
            }
            if (_inertia > 0) {
                _angularVelocity *= 1 / _inertia
            }
            timestamp = system!!.timestamp
        }
    }
}
