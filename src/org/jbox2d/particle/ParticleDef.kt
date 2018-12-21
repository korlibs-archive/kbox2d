package org.jbox2d.particle

import org.jbox2d.common.Vec2

class ParticleDef {
    /**
     * Specifies the type of particle. A particle may be more than one type. Multiple types are
     * chained by logical sums, for example: pd.flags = ParticleType.b2_elasticParticle |
     * ParticleType.b2_viscousParticle.
     */
    @JvmField
    internal var flags: Int = 0

    /** The world position of the particle.  */
    @JvmField
    val position = Vec2()

    /** The linear velocity of the particle in world co-ordinates.  */
    @JvmField
    val velocity = Vec2()

    /** The color of the particle.  */
    @JvmField
    var color: ParticleColor? = null

    /** Use this to store application-specific body data.  */
    @JvmField
    var userData: Any? = null
}
