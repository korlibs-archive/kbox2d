package org.jbox2d.particle

/**
 * The particle type. Can be combined with | operator. Zero means liquid.
 *
 * @author dmurph
 */
object ParticleType {

    val waterParticle = 0

    /** removed after next step  */
    val zombieParticle = 1 shl 1

    /** zero velocity  */
    val wallParticle = 1 shl 2

    /** with restitution from stretching  */
    val springParticle = 1 shl 3

    /** with restitution from deformation  */
    val elasticParticle = 1 shl 4

    /** with viscosity  */
    val viscousParticle = 1 shl 5

    /** without isotropic pressure  */
    val powderParticle = 1 shl 6

    /** with surface tension  */
    val tensileParticle = 1 shl 7

    /** mixing color between contacting particles  */
    val colorMixingParticle = 1 shl 8

    /** call b2DestructionListener on destruction  */
    val destructionListener = 1 shl 9
}
