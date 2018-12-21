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
package org.jbox2d.common

import java.io.Serializable

/**
 * A 2D column vector
 */
class Vec2 @JvmOverloads constructor(
        @JvmField var x: Float = 0f,
        @JvmField var y: Float = 0f
) : Serializable {

    //@Deprecated("", ReplaceWith("x"))
    //fun getX() = x
    //@Deprecated("", ReplaceWith("y"))
    //fun getY() = y
    //
    //fun setX(value: Float) {
    //    this.x = value
    //}
    //
    //fun setY(value: Float) {
    //    this.y = value
    //}

    /** True if the vector represents a pair of valid, non-infinite floating point numbers.  */
    val isValid: Boolean
        get() = !java.lang.Float.isNaN(x) && !java.lang.Float.isInfinite(x) && !java.lang.Float.isNaN(y) && !java.lang.Float.isInfinite(y)

    constructor(toCopy: Vec2) : this(toCopy.x, toCopy.y) {}

    /** Zero out this vector.  */
    fun setZero() {
        x = 0.0f
        y = 0.0f
    }

    /** Set the vector component-wise.  */
    operator fun set(x: Float, y: Float): Vec2 {
        this.x = x
        this.y = y
        return this
    }

    /** Set this vector to another vector.  */
    fun set(v: Vec2): Vec2 {
        this.x = v.x
        this.y = v.y
        return this
    }

    /** Return the sum of this vector and another; does not alter either one.  */
    fun add(v: Vec2): Vec2 {
        return Vec2(x + v.x, y + v.y)
    }


    /** Return the difference of this vector and another; does not alter either one.  */
    fun sub(v: Vec2): Vec2 {
        return Vec2(x - v.x, y - v.y)
    }

    /** Return this vector multiplied by a scalar; does not alter this vector.  */
    fun mul(a: Float): Vec2 {
        return Vec2(x * a, y * a)
    }

    /** Return the negation of this vector; does not alter this vector.  */
    fun negate(): Vec2 {
        return Vec2(-x, -y)
    }

    /** Flip the vector and return it - alters this vector.  */
    fun negateLocal(): Vec2 {
        x = -x
        y = -y
        return this
    }

    /** Add another vector to this one and returns result - alters this vector.  */
    fun addLocal(v: Vec2): Vec2 {
        x += v.x
        y += v.y
        return this
    }

    /** Adds values to this vector and returns result - alters this vector.  */
    fun addLocal(x: Float, y: Float): Vec2 {
        this.x += x
        this.y += y
        return this
    }

    /** Subtract another vector from this one and return result - alters this vector.  */
    fun subLocal(v: Vec2): Vec2 {
        x -= v.x
        y -= v.y
        return this
    }

    /** Multiply this vector by a number and return result - alters this vector.  */
    fun mulLocal(a: Float): Vec2 {
        x *= a
        y *= a
        return this
    }

    /** Get the skew vector such that dot(skew_vec, other) == cross(vec, other)  */
    fun skew(): Vec2 {
        return Vec2(-y, x)
    }

    /** Get the skew vector such that dot(skew_vec, other) == cross(vec, other)  */
    fun skew(out: Vec2) {
        out.x = -y
        out.y = x
    }

    /** Return the length of this vector.  */
    fun length(): Float {
        return MathUtils.sqrt(x * x + y * y)
    }

    /** Return the squared length of this vector.  */
    fun lengthSquared(): Float {
        return x * x + y * y
    }

    /** Normalize this vector and return the length before normalization. Alters this vector.  */
    fun normalize(): Float {
        val length = length()
        if (length < Settings.EPSILON) {
            return 0f
        }

        val invLength = 1.0f / length
        x *= invLength
        y *= invLength
        return length
    }

    /** Return a new vector that has positive components.  */
    fun abs(): Vec2 {
        return Vec2(MathUtils.abs(x), MathUtils.abs(y))
    }

    fun absLocal() {
        x = MathUtils.abs(x)
        y = MathUtils.abs(y)
    }

    // @Override // annotation omitted for GWT-compatibility
    /** Return a copy of this vector.  */
    fun clone(): Vec2 {
        return Vec2(x, y)
    }

    override fun toString(): String {
        return "($x,$y)"
    }

    /**
     * @see java.lang.Object.hashCode
     */
    override fun hashCode(): Int { // automatically generated by Eclipse
        val prime = 31
        var result = 1
        result = prime * result + java.lang.Float.floatToIntBits(x)
        result = prime * result + java.lang.Float.floatToIntBits(y)
        return result
    }

    /**
     * @see java.lang.Object.equals
     */
    override fun equals(obj: Any?): Boolean { // automatically generated by Eclipse
        if (this === obj) return true
        if (obj == null) return false
        if (javaClass != obj.javaClass) return false
        val other = obj as Vec2?
        if (java.lang.Float.floatToIntBits(x) != java.lang.Float.floatToIntBits(other!!.x)) return false
        return if (java.lang.Float.floatToIntBits(y) != java.lang.Float.floatToIntBits(other.y)) false else true
    }

    companion object {
        private const val serialVersionUID = 1L

        @JvmStatic
        fun abs(a: Vec2): Vec2 {
            return Vec2(MathUtils.abs(a.x), MathUtils.abs(a.y))
        }

        @JvmStatic
        fun absToOut(a: Vec2, out: Vec2) {
            out.x = MathUtils.abs(a.x)
            out.y = MathUtils.abs(a.y)
        }

        @JvmStatic
        fun dot(a: Vec2, b: Vec2): Float {
            return a.x * b.x + a.y * b.y
        }

        @JvmStatic
        fun cross(a: Vec2, b: Vec2): Float {
            return a.x * b.y - a.y * b.x
        }

        @JvmStatic
        fun cross(a: Vec2, s: Float): Vec2 {
            return Vec2(s * a.y, -s * a.x)
        }

        @JvmStatic
        fun crossToOut(a: Vec2, s: Float, out: Vec2) {
            val tempy = -s * a.x
            out.x = s * a.y
            out.y = tempy
        }

        @JvmStatic
        fun crossToOutUnsafe(a: Vec2, s: Float, out: Vec2) {
            assert(out !== a)
            out.x = s * a.y
            out.y = -s * a.x
        }

        @JvmStatic
        fun cross(s: Float, a: Vec2): Vec2 {
            return Vec2(-s * a.y, s * a.x)
        }

        @JvmStatic
        fun crossToOut(s: Float, a: Vec2, out: Vec2) {
            val tempY = s * a.x
            out.x = -s * a.y
            out.y = tempY
        }

        @JvmStatic
        fun crossToOutUnsafe(s: Float, a: Vec2, out: Vec2) {
            assert(out !== a)
            out.x = -s * a.y
            out.y = s * a.x
        }

        @JvmStatic
        fun negateToOut(a: Vec2, out: Vec2) {
            out.x = -a.x
            out.y = -a.y
        }

        @JvmStatic
        fun min(a: Vec2, b: Vec2): Vec2 {
            return Vec2(if (a.x < b.x) a.x else b.x, if (a.y < b.y) a.y else b.y)
        }

        @JvmStatic
        fun max(a: Vec2, b: Vec2): Vec2 {
            return Vec2(if (a.x > b.x) a.x else b.x, if (a.y > b.y) a.y else b.y)
        }

        @JvmStatic
        fun minToOut(a: Vec2, b: Vec2, out: Vec2) {
            out.x = if (a.x < b.x) a.x else b.x
            out.y = if (a.y < b.y) a.y else b.y
        }

        @JvmStatic
        fun maxToOut(a: Vec2, b: Vec2, out: Vec2) {
            out.x = if (a.x > b.x) a.x else b.x
            out.y = if (a.y > b.y) a.y else b.y
        }
    }
}
