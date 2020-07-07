package org.jbox2d.particle

import org.jbox2d.common.MathUtils
import org.jbox2d.common.Vec2
import org.jbox2d.internal.*
import org.jbox2d.pooling.normal.MutableStack

class VoronoiDiagram(generatorCapacity: Int) {

    private val generatorBuffer: Array<Generator> = Array(generatorCapacity) { Generator() }
    private var generatorCount: Int = 0
    private var countX: Int = 0
    private var countY: Int = 0
    // The diagram is an array of "pointers".
    private var diagram: Array<Generator?>? = null

    private val lower = Vec2()
    private val upper = Vec2()
    /*
    e: java.lang.IllegalStateException: Cannot get FQ name of local class: class <no name provided> : org.jbox2d.pooling.normal.MutableStack<org.jbox2d.particle.VoronoiDiagram.VoronoiDiagramTask> defined in private final val taskPool: <no name provided> defined in org.jbox2d.particle.VoronoiDiagram
        at org.jetbrains.kotlin.serialization.DescriptorAwareStringTable$DefaultImpls.getFqNameIndex(DescriptorAwareStringTable.kt:23)
        at org.jetbrains.kotlin.serialization.StringTableImpl.getFqNameIndex(StringTableImpl.kt:25)
        at org.jetbrains.kotlin.serialization.DescriptorSerializer.getClassifierId(DescriptorSerializer.kt:718)
        at org.jetbrains.kotlin.serialization.DescriptorSerializer.fillFromPossiblyInnerType(DescriptorSerializer.kt:590)
        at org.jetbrains.kotlin.serialization.DescriptorSerializer.type$serialization(DescriptorSerializer.kt:555)
        at org.jetbrains.kotlin.serialization.DescriptorSerializer.typeId$serialization(DescriptorSerializer.kt:520)
        at org.jetbrains.kotlin.serialization.DescriptorSerializer.propertyProto(DescriptorSerializer.kt:242)
     */
    //private val taskPool = object : MutableStack<VoronoiDiagram.VoronoiDiagramTask>(50) {
    //    override fun newInstance(): VoronoiDiagramTask = VoronoiDiagramTask()
    //    override fun newArray(size: Int): Array<VoronoiDiagramTask> =
    //        arrayOfNulls<VoronoiDiagramTask>(size) as Array<VoronoiDiagramTask>
    //}

    val taskPool: VoronoiDiagramTaskMutableStack = VoronoiDiagramTaskMutableStack()

    class VoronoiDiagramTaskMutableStack : MutableStack<VoronoiDiagramTask>(50) {
        override fun newInstance(): VoronoiDiagramTask = VoronoiDiagramTask()
        override fun newArray(size: Int): Array<VoronoiDiagramTask> =
            arrayOfNulls<VoronoiDiagramTask>(size) as Array<VoronoiDiagramTask>
    }

    private val queue = StackQueue<VoronoiDiagramTask>()

    class Generator {
        internal val center = Vec2()
        internal var tag: Int = 0
    }

    class VoronoiDiagramTask {
        internal var x: Int = 0
        internal var y: Int = 0
        internal var i: Int = 0
        internal var generator: Generator? = null

        constructor() {}

        constructor(x: Int, y: Int, i: Int, g: Generator) {
            this.x = x
            this.y = y
            this.i = i
            generator = g
        }

        fun set(x: Int, y: Int, i: Int, g: Generator): VoronoiDiagramTask {
            this.x = x
            this.y = y
            this.i = i
            generator = g
            return this
        }
    }

    interface VoronoiDiagramCallback {
        fun callback(aTag: Int, bTag: Int, cTag: Int)
    }

    fun getNodes(callback: VoronoiDiagramCallback) {
        for (y in 0 until countY - 1) {
            for (x in 0 until countX - 1) {
                val i = x + y * countX
                val a = diagram!![i]
                val b = diagram!![i + 1]
                val c = diagram!![i + countX]
                val d = diagram!![i + 1 + countX]
                if (b !== c) {
                    if (a !== b && a !== c) {
                        callback.callback(a!!.tag, b!!.tag, c!!.tag)
                    }
                    if (d !== b && d !== c) {
                        callback.callback(b!!.tag, d!!.tag, c!!.tag)
                    }
                }
            }
        }
    }

    fun addGenerator(center: Vec2, tag: Int) {
        val g = generatorBuffer[generatorCount++]
        g.center.x = center.x
        g.center.y = center.y
        g.tag = tag
    }

    fun generate(radius: Float) {
        assert(diagram == null)
        val inverseRadius = 1 / radius
        lower.x = Float.MAX_VALUE
        lower.y = Float.MAX_VALUE
        upper.x = -Float.MAX_VALUE
        upper.y = -Float.MAX_VALUE
        for (k in 0 until generatorCount) {
            val g = generatorBuffer[k]
            Vec2.minToOut(lower, g.center, lower)
            Vec2.maxToOut(upper, g.center, upper)
        }
        countX = 1 + (inverseRadius * (upper.x - lower.x)).toInt()
        countY = 1 + (inverseRadius * (upper.y - lower.y)).toInt()
        diagram = arrayOfNulls<Generator>(countX * countY)
        queue.reset(arrayOfNulls<VoronoiDiagramTask>(4 * countX * countX) as Array<VoronoiDiagramTask>)
        for (k in 0 until generatorCount) {
            val g = generatorBuffer[k]
            g.center.x = inverseRadius * (g.center.x - lower.x)
            g.center.y = inverseRadius * (g.center.y - lower.y)
            val x = MathUtils.max(0, MathUtils.min(g.center.x.toInt(), countX - 1))
            val y = MathUtils.max(0, MathUtils.min(g.center.y.toInt(), countY - 1))
            queue.push(taskPool.pop().set(x, y, x + y * countX, g))
        }
        while (!queue.empty()) {
            val front = queue.pop()
            val x = front.x
            val y = front.y
            val i = front.i
            val g = front.generator
            if (diagram!![i] == null) {
                diagram!![i] = g!!
                if (x > 0) {
                    queue.push(taskPool.pop().set(x - 1, y, i - 1, g))
                }
                if (y > 0) {
                    queue.push(taskPool.pop().set(x, y - 1, i - countX, g))
                }
                if (x < countX - 1) {
                    queue.push(taskPool.pop().set(x + 1, y, i + 1, g))
                }
                if (y < countY - 1) {
                    queue.push(taskPool.pop().set(x, y + 1, i + countX, g))
                }
            }
            taskPool.push(front)
        }
        val maxIteration = countX + countY
        for (iteration in 0 until maxIteration) {
            for (y in 0 until countY) {
                for (x in 0 until countX - 1) {
                    val i = x + y * countX
                    val a = diagram!![i]!!
                    val b = diagram!![i + 1]!!
                    if (a !== b) {
                        queue.push(taskPool.pop().set(x, y, i, b))
                        queue.push(taskPool.pop().set(x + 1, y, i + 1, a))
                    }
                }
            }
            for (y in 0 until countY - 1) {
                for (x in 0 until countX) {
                    val i = x + y * countX
                    val a = diagram!![i]!!
                    val b = diagram!![i + countX]!!
                    if (a !== b) {
                        queue.push(taskPool.pop().set(x, y, i, b))
                        queue.push(taskPool.pop().set(x, y + 1, i + countX, a))
                    }
                }
            }
            var updated = false
            while (!queue.empty()) {
                val front = queue.pop()
                val x = front.x
                val y = front.y
                val i = front.i
                val k = front.generator
                val a = diagram!![i]
                val b = k
                if (a !== b) {
                    val ax = a!!.center.x - x
                    val ay = a.center.y - y
                    val bx = b!!.center.x - x
                    val by = b.center.y - y
                    val a2 = ax * ax + ay * ay
                    val b2 = bx * bx + by * by
                    if (a2 > b2) {
                        diagram!![i] = b
                        if (x > 0) {
                            queue.push(taskPool.pop().set(x - 1, y, i - 1, b))
                        }
                        if (y > 0) {
                            queue.push(taskPool.pop().set(x, y - 1, i - countX, b))
                        }
                        if (x < countX - 1) {
                            queue.push(taskPool.pop().set(x + 1, y, i + 1, b))
                        }
                        if (y < countY - 1) {
                            queue.push(taskPool.pop().set(x, y + 1, i + countX, b))
                        }
                        updated = true
                    }
                }
                taskPool.push(front)
            }
            if (!updated) break
        }
    }
}
