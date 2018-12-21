package org.jbox2d.common

object BufferUtils {
    /** Reallocate a buffer.  */
    @JvmStatic
    fun <T> reallocateBuffer(klass: Class<T>, oldBuffer: Array<*>?, oldCapacity: Int,
                             newCapacity: Int): Array<T> {
        assert(newCapacity > oldCapacity)
        val newBuffer = java.lang.reflect.Array.newInstance(klass, newCapacity) as Array<T>
        if (oldBuffer != null) {
            System.arraycopy(oldBuffer!!, 0, newBuffer, 0, oldCapacity)
        }
        for (i in oldCapacity until newCapacity) {
            try {
                newBuffer[i] = klass.newInstance()
            } catch (e: Exception) {
                throw RuntimeException(e)
            }

        }
        return newBuffer
    }

    /** Reallocate a buffer.  */
    @JvmStatic
    fun reallocateBuffer(oldBuffer: IntArray?, oldCapacity: Int, newCapacity: Int): IntArray {
        assert(newCapacity > oldCapacity)
        val newBuffer = IntArray(newCapacity)
        if (oldBuffer != null) {
            System.arraycopy(oldBuffer, 0, newBuffer, 0, oldCapacity)
        }
        return newBuffer
    }

    /** Reallocate a buffer.  */
    @JvmStatic
    fun reallocateBuffer(oldBuffer: FloatArray?, oldCapacity: Int, newCapacity: Int): FloatArray {
        assert(newCapacity > oldCapacity)
        val newBuffer = FloatArray(newCapacity)
        if (oldBuffer != null) {
            System.arraycopy(oldBuffer, 0, newBuffer, 0, oldCapacity)
        }
        return newBuffer
    }

    /**
     * Reallocate a buffer. A 'deferred' buffer is reallocated only if it is not NULL. If
     * 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
     */
    @JvmStatic
    fun <T> reallocateBuffer(klass: Class<T>, buffer: Array<T>?, userSuppliedCapacity: Int,
                             oldCapacity: Int, newCapacity: Int, deferred: Boolean): Array<T> {
        var buffer = buffer
        assert(newCapacity > oldCapacity)
        assert(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity)
        if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            buffer = reallocateBuffer(klass, buffer, oldCapacity, newCapacity)
        }
        return buffer!!
    }

    /**
     * Reallocate an int buffer. A 'deferred' buffer is reallocated only if it is not NULL. If
     * 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
     */
    @JvmStatic
    fun reallocateBuffer(buffer: IntArray?, userSuppliedCapacity: Int, oldCapacity: Int,
                         newCapacity: Int, deferred: Boolean): IntArray {
        var buffer = buffer
        assert(newCapacity > oldCapacity)
        assert(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity)
        if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            buffer = reallocateBuffer(buffer, oldCapacity, newCapacity)
        }
        return buffer!!
    }

    /**
     * Reallocate a float buffer. A 'deferred' buffer is reallocated only if it is not NULL. If
     * 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
     */
    @JvmStatic
    fun reallocateBuffer(buffer: FloatArray?, userSuppliedCapacity: Int, oldCapacity: Int,
                         newCapacity: Int, deferred: Boolean): FloatArray {
        var buffer = buffer
        assert(newCapacity > oldCapacity)
        assert(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity)
        if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
            buffer = reallocateBuffer(buffer, oldCapacity, newCapacity)
        }
        return buffer!!
    }

    /** Rotate an array, see std::rotate  */
    @JvmStatic
    fun <T> rotate(ray: Array<T>, first: Int, new_first: Int, last: Int) {
        var first = first
        var new_first = new_first
        var next = new_first
        while (next != first) {
            val temp = ray[first]
            ray[first] = ray[next]
            ray[next] = temp
            first++
            next++
            if (next == last) {
                next = new_first
            } else if (first == new_first) {
                new_first = next
            }
        }
    }

    /** Rotate an array, see std::rotate  */
    @JvmStatic
    fun rotate(ray: IntArray, first: Int, new_first: Int, last: Int) {
        var first = first
        var new_first = new_first
        var next = new_first
        while (next != first) {
            val temp = ray[first]
            ray[first] = ray[next]
            ray[next] = temp
            first++
            next++
            if (next == last) {
                next = new_first
            } else if (first == new_first) {
                new_first = next
            }
        }
    }

    /** Rotate an array, see std::rotate  */
    @JvmStatic
    fun rotate(ray: FloatArray, first: Int, new_first: Int, last: Int) {
        var first = first
        var new_first = new_first
        var next = new_first
        while (next != first) {
            val temp = ray[first]
            ray[first] = ray[next]
            ray[next] = temp
            first++
            next++
            if (next == last) {
                next = new_first
            } else if (first == new_first) {
                new_first = next
            }
        }
    }
}
