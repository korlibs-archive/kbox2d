package org.jbox2d.particle

/**
 * The particle type. Can be combined with | operator. Zero means liquid.
 *
 * @author dmurph
 */
object ParticleType {
    @JvmField
    val b2_waterParticle = 0
    /** removed after next step  */
    @JvmField
    val b2_zombieParticle = 1 shl 1
    /** zero velocity  */
    @JvmField
    val b2_wallParticle = 1 shl 2
    /** with restitution from stretching  */
    @JvmField
    val b2_springParticle = 1 shl 3
    /** with restitution from deformation  */
    @JvmField
    val b2_elasticParticle = 1 shl 4
    /** with viscosity  */
    @JvmField
    val b2_viscousParticle = 1 shl 5
    /** without isotropic pressure  */
    @JvmField
    val b2_powderParticle = 1 shl 6
    /** with surface tension  */
    @JvmField
    val b2_tensileParticle = 1 shl 7
    /** mixing color between contacting particles  */
    @JvmField
    val b2_colorMixingParticle = 1 shl 8
    /** call b2DestructionListener on destruction  */
    @JvmField
    val b2_destructionListener = 1 shl 9
}
