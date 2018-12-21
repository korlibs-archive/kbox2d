package org.jbox2d.particle

object ParticleGroupType {
    /** resists penetration  */
    @JvmField
    val b2_solidParticleGroup = 1 shl 0
    /** keeps its shape  */
    @JvmField
    val b2_rigidParticleGroup = 1 shl 1
}
