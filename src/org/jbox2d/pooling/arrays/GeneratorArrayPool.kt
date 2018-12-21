package org.jbox2d.pooling.arrays

import java.util.HashMap

import org.jbox2d.particle.VoronoiDiagram

class GeneratorArrayPool {

    private val map = HashMap<Int, Array<VoronoiDiagram.Generator>>()

    operator fun get(length: Int): Array<VoronoiDiagram.Generator> {
        assert(length > 0)

        if (!map.containsKey(length)) {
            map[length] = getInitializedArray(length)
        }

        assert(map[length]!!.size == length) { "Array not built of correct length" }
        return map[length]!!
    }

    protected fun getInitializedArray(length: Int): Array<VoronoiDiagram.Generator> {
        return Array(length) { VoronoiDiagram.Generator() }
    }
}
