package org.jbox2d.particle

import org.jbox2d.collision.shapes.Shape
import org.jbox2d.common.Vec2

/**
 * A particle group definition holds all the data needed to construct a particle group. You can
 * safely re-use these definitions.
 */
class ParticleGroupDef {

    /** The particle-behavior flags.  */
    @JvmField
    var flags: Int = 0

    /** The group-construction flags.  */
    @JvmField
    var groupFlags: Int = 0

    /**
     * The world position of the group. Moves the group's shape a distance equal to the value of
     * position.
     */
    @JvmField
    val position = Vec2()

    /**
     * The world angle of the group in radians. Rotates the shape by an angle equal to the value of
     * angle.
     */
    @JvmField
    var angle: Float = 0f

    /** The linear velocity of the group's origin in world co-ordinates.  */
    @JvmField
    val linearVelocity = Vec2()

    /** The angular velocity of the group.  */
    @JvmField
    var angularVelocity: Float = 0f

    /** The color of all particles in the group.  */
    @JvmField
    var color: ParticleColor? = null

    /**
     * The strength of cohesion among the particles in a group with flag b2_elasticParticle or
     * b2_springParticle.
     */
    @JvmField
    var strength: Float = 1f

    /** Shape containing the particle group.  */
    @JvmField
    var shape: Shape? = null

    /** If true, destroy the group automatically after its last particle has been destroyed.  */
    @JvmField
    var destroyAutomatically: Boolean = true

    /** Use this to store application-specific group data.  */
    @JvmField
    var userData: Any? = null
}
