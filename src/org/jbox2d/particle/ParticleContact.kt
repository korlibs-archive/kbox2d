package org.jbox2d.particle

import org.jbox2d.common.Vec2

class ParticleContact {
    /** Indices of the respective particles making contact.  */
    @JvmField
    var indexA: Int = 0
    @JvmField
    var indexB: Int = 0
    /** The logical sum of the particle behaviors that have been set.  */
    @JvmField
    var flags: Int = 0
    /** Weight of the contact. A value between 0.0f and 1.0f.  */
    @JvmField
    var weight: Float = 0.toFloat()
    /** The normalized direction from A to B.  */
    @JvmField
    val normal = Vec2()
}
