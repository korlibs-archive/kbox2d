package org.jbox2d.particle

import org.jbox2d.internal.*

class StackQueue<T> {

    private var buffer: Array<T>? = null
    private var front: Int = 0
    private var back: Int = 0
    private var end: Int = 0

    fun reset(buffer: Array<T>) {
        this.buffer = buffer
        front = 0
        back = 0
        end = buffer.size
    }

    fun push(task: T) {
        if (back >= end) {
            arraycopy(buffer!!, front, buffer!!, 0, back - front)
            back -= front
            front = 0
            if (back >= end) return
        }
        buffer!![back++] = task
    }

    fun pop(): T {
        assert(front < back)
        return buffer!![front++]
    }

    fun empty(): Boolean {
        return front >= back
    }

    fun front(): T {
        return buffer!![front]
    }
}
