package org.jbox2d.particle

import org.jbox2d.callbacks.ParticleQueryCallback
import org.jbox2d.callbacks.ParticleRaycastCallback
import org.jbox2d.callbacks.QueryCallback
import org.jbox2d.collision.AABB
import org.jbox2d.collision.RayCastInput
import org.jbox2d.collision.RayCastOutput
import org.jbox2d.collision.shapes.Shape
import org.jbox2d.common.BufferUtils
import org.jbox2d.common.MathUtils
import org.jbox2d.common.Rot
import org.jbox2d.common.Settings
import org.jbox2d.common.Transform
import org.jbox2d.common.Vec2
import org.jbox2d.dynamics.Fixture
import org.jbox2d.dynamics.TimeStep
import org.jbox2d.dynamics.World
import org.jbox2d.internal.*
import org.jbox2d.particle.VoronoiDiagram.VoronoiDiagramCallback
import kotlin.experimental.and

class ParticleSystem(internal var world: World) {

    internal var timestamp: Int = 0
    internal var allParticleFlags: Int = 0
    internal var allGroupFlags: Int = 0
    internal var density: Float = 1f
    internal var inverseDensity: Float = 1f
    var particleGravityScale: Float = 1f
    internal var particleDiameter: Float = 1f
    internal var inverseDiameter: Float = 1f
    internal var squaredDiameter: Float = 1f

    var particleCount: Int = 0
        internal set
    internal var internalAllocatedCapacity: Int = 0
    internal var maxCount: Int = 0
    internal var flagsBuffer: ParticleBufferInt = ParticleBufferInt()
    internal var positionBuffer: ParticleBuffer<Vec2> = ParticleBuffer { Vec2() }
    internal var velocityBuffer: ParticleBuffer<Vec2> = ParticleBuffer { Vec2() }
    internal var accumulationBuffer: FloatArray = FloatArray(0) // temporary values
    internal var accumulation2Buffer: Array<Vec2> = emptyArray() // temporary vector values
    internal var depthBuffer: FloatArray? = null // distance from the surface

    var colorBuffer: ParticleBuffer<ParticleColor> = ParticleBuffer { ParticleColor() }
    var particleGroupBuffer: Array<ParticleGroup?> = emptyArray()
        internal set
    internal var userDataBuffer: ParticleBuffer<Any> = ParticleBuffer { Any() }

    internal var proxyCount: Int = 0
    internal var proxyCapacity: Int = 0
    internal var proxyBuffer: Array<Proxy> = emptyArray()

    var contactCount: Int = 0
    internal var contactCapacity: Int = 0
    var contactBuffer: Array<ParticleContact> = emptyArray()

    var bodyContactCount: Int = 0
    internal var bodyContactCapacity: Int = 0
    var bodyContactBuffer: Array<ParticleBodyContact> = emptyArray()

    internal var pairCount: Int = 0
    internal var pairCapacity: Int = 0
    internal var pairBuffer: Array<Pair> = emptyArray()

    internal var triadCount: Int = 0
    internal var triadCapacity: Int = 0
    internal var triadBuffer: Array<Triad> = emptyArray()

    var particleGroupCount: Int = 0
        internal set
    internal var groupList: ParticleGroup? = null

    internal var pressureStrength: Float = 0.05f
    var particleDamping: Float = 1f
    internal var elasticStrength: Float = 0.25f
    internal var springStrength: Float = 0.25f
    internal var viscousStrength: Float = 0.25f
    internal var surfaceTensionStrengthA: Float = 0.1f
    internal var surfaceTensionStrengthB: Float = 0.2f
    internal var powderStrength: Float = 0.5f
    internal var ejectionStrength: Float = 0.5f
    internal var colorMixingStrength: Float = 0.5f

    private val temp = AABB()
    private val dpcallback = DestroyParticlesInShapeCallback()

    private val temp2 = AABB()
    private val tempVec = Vec2()
    private val tempTransform = Transform()
    private val tempTransform2 = Transform()
    private val createParticleGroupCallback = CreateParticleGroupCallback()
    private val tempParticleDef = ParticleDef()

    private val ubccallback = UpdateBodyContactsCallback()

    private val sccallback = SolveCollisionCallback()

    private val tempVec2 = Vec2()
    private val tempRot = Rot()
    private val tempXf = Transform()
    private val tempXf2 = Transform()

    private val newIndices = NewIndices()

    var particleDensity: Float
        get() = density
        set(density) {
            this.density = density
            inverseDensity = 1 / this.density
        }

    var particleRadius: Float
        get() = particleDiameter / 2
        set(radius) {
            particleDiameter = 2 * radius
            squaredDiameter = particleDiameter * particleDiameter
            inverseDiameter = 1 / particleDiameter
        }

    internal val particleStride: Float
        get() = Settings.particleStride * particleDiameter

    internal val particleMass: Float
        get() {
            val stride = particleStride
            return density * stride * stride
        }

    internal val particleInvMass: Float
        get() = 1.777777f * inverseDensity * inverseDiameter * inverseDiameter

    val particleFlagsBuffer: IntArray?
        get() = flagsBuffer.data

    val particlePositionBuffer: Array<Vec2>?
        get() = positionBuffer.data

    val particleVelocityBuffer: Array<Vec2>?
        get() = velocityBuffer.data

    val particleColorBuffer: Array<ParticleColor>?
        get() {
            colorBuffer.data = requestParticleBuffer({ ParticleColor() }, colorBuffer.data)
            return colorBuffer.data
        }

    val particleUserDataBuffer: Array<Any>?
        get() {
            userDataBuffer.data = requestParticleBuffer({ Any() }, userDataBuffer.data)
            return userDataBuffer.data
        }

    var particleMaxCount: Int
        get() = maxCount
        set(count) {
            assert(particleCount <= count)
            maxCount = count
        }

    //  public void assertNotSamePosition() {
    //    for (int i = 0; i < m_count; i++) {
    //      Vec2 vi = m_positionBuffer.data[i];
    //      for (int j = i + 1; j < m_count; j++) {
    //        Vec2 vj = m_positionBuffer.data[j];
    //        assert(vi.x != vj.x || vi.y != vj.y);
    //      }
    //    }
    //  }

    fun createParticle(def: ParticleDef): Int {
        if (particleCount >= internalAllocatedCapacity) {
            var capacity = if (particleCount != 0) 2 * particleCount else Settings.minParticleBufferCapacity
            capacity = limitCapacity(capacity, maxCount)
            capacity = limitCapacity(capacity, flagsBuffer.userSuppliedCapacity)
            capacity = limitCapacity(capacity, positionBuffer.userSuppliedCapacity)
            capacity = limitCapacity(capacity, velocityBuffer.userSuppliedCapacity)
            capacity = limitCapacity(capacity, colorBuffer.userSuppliedCapacity)
            capacity = limitCapacity(capacity, userDataBuffer.userSuppliedCapacity)
            if (internalAllocatedCapacity < capacity) {
                flagsBuffer.data = reallocateBuffer(flagsBuffer, internalAllocatedCapacity, capacity, false)
                positionBuffer.data = reallocateBuffer(positionBuffer, internalAllocatedCapacity, capacity, false)
                velocityBuffer.data = reallocateBuffer(velocityBuffer, internalAllocatedCapacity, capacity, false)
                accumulationBuffer =
                    BufferUtils.reallocateBuffer(accumulationBuffer, 0, internalAllocatedCapacity, capacity, false)
                accumulation2Buffer = BufferUtils.reallocateBuffer(
                    { Vec2() },
                    accumulation2Buffer,
                    0,
                    internalAllocatedCapacity,
                    capacity,
                    true
                )
                depthBuffer = BufferUtils.reallocateBuffer(depthBuffer, 0, internalAllocatedCapacity, capacity, true)
                colorBuffer.data = reallocateBuffer(colorBuffer, internalAllocatedCapacity, capacity, true)
                particleGroupBuffer = BufferUtils.reallocateBuffer<ParticleGroup>(
                    { ParticleGroup() },
                    particleGroupBuffer as Array<ParticleGroup>,
                    0,
                    internalAllocatedCapacity,
                    capacity,
                    false
                ) as Array<ParticleGroup?>
                userDataBuffer.data = reallocateBuffer(userDataBuffer, internalAllocatedCapacity, capacity, true)
                internalAllocatedCapacity = capacity
            }
        }
        if (particleCount >= internalAllocatedCapacity) {
            return Settings.invalidParticleIndex
        }
        val index = particleCount++
        flagsBuffer.data!![index] = def.flags
        positionBuffer.data!![index].set(def.position)
        //    assertNotSamePosition();
        velocityBuffer.data!![index].set(def.velocity)
        particleGroupBuffer[index] = null
        if (depthBuffer != null) {
            depthBuffer!![index] = 0f
        }
        if (colorBuffer.data != null || def.color != null) {
            colorBuffer.data = requestParticleBuffer(colorBuffer.dataClass, colorBuffer.data)
            colorBuffer.data!![index].set(def.color!!)
        }
        if (userDataBuffer.data != null || def.userData != null) {
            userDataBuffer.data = requestParticleBuffer(userDataBuffer.dataClass, userDataBuffer.data)
            userDataBuffer.data!![index] = def.userData!!
        }
        if (proxyCount >= proxyCapacity) {
            val oldCapacity = proxyCapacity
            val newCapacity = if (proxyCount != 0) 2 * proxyCount else Settings.minParticleBufferCapacity
            proxyBuffer = BufferUtils.reallocateBuffer({ Proxy() }, proxyBuffer, oldCapacity, newCapacity)
            proxyCapacity = newCapacity
        }
        proxyBuffer[proxyCount++].index = index
        return index
    }

    fun destroyParticle(index: Int, callDestructionListener: Boolean) {
        var flags = ParticleType.zombieParticle
        if (callDestructionListener) {
            flags = flags or ParticleType.destructionListener
        }
        flagsBuffer.data!![index] = flagsBuffer.data!![index] or flags
    }

    fun destroyParticlesInShape(shape: Shape, xf: Transform, callDestructionListener: Boolean): Int {
        dpcallback.init(this, shape, xf, callDestructionListener)
        shape.computeAABB(temp, xf, 0)
        world.queryAABB(dpcallback, temp)
        return dpcallback.destroyed
    }

    fun destroyParticlesInGroup(group: ParticleGroup, callDestructionListener: Boolean) {
        for (i in group.firstIndex until group.lastIndex) {
            destroyParticle(i, callDestructionListener)
        }
    }

    fun createParticleGroup(groupDef: ParticleGroupDef): ParticleGroup {
        val stride = particleStride
        val identity = tempTransform
        identity.setIdentity()
        val transform = tempTransform2
        transform.setIdentity()
        val firstIndex = particleCount
        if (groupDef.shape != null) {
            val particleDef = tempParticleDef
            particleDef.flags = groupDef.flags
            particleDef.color = groupDef.color
            particleDef.userData = groupDef.userData
            val shape = groupDef.shape
            transform.setRadians(groupDef.position, groupDef.angleRadians)
            val aabb = temp
            val childCount = shape!!.getChildCount()
            for (childIndex in 0 until childCount) {
                if (childIndex == 0) {
                    shape.computeAABB(aabb, identity, childIndex)
                } else {
                    val childAABB = temp2
                    shape.computeAABB(childAABB, identity, childIndex)
                    aabb.combine(childAABB)
                }
            }
            val upperBoundY = aabb.upperBound.y
            val upperBoundX = aabb.upperBound.x
            var y = MathUtils.floor(aabb.lowerBound.y / stride) * stride
            while (y < upperBoundY) {
                var x = MathUtils.floor(aabb.lowerBound.x / stride) * stride
                while (x < upperBoundX) {
                    val p = tempVec
                    p.x = x
                    p.y = y
                    if (shape.testPoint(identity, p)) {
                        Transform.mulToOut(transform, p, p)
                        particleDef.position.x = p.x
                        particleDef.position.y = p.y
                        p.subLocal(groupDef.position)
                        Vec2.crossToOutUnsafe(groupDef.angularVelocity, p, particleDef.velocity)
                        particleDef.velocity.addLocal(groupDef.linearVelocity)
                        createParticle(particleDef)
                    }
                    x += stride
                }
                y += stride
            }
        }
        val lastIndex = particleCount

        val group = ParticleGroup()
        group.system = this
        group.firstIndex = firstIndex
        group.lastIndex = lastIndex
        group.groupFlags = groupDef.groupFlags
        group.strength = groupDef.strength
        group.userData = groupDef.userData
        group.transform.set(transform)
        group.destroyAutomatically = groupDef.destroyAutomatically
        group.prev = null
        group.next = groupList
        if (groupList != null) {
            groupList!!.prev = group
        }
        groupList = group
        ++particleGroupCount
        for (i in firstIndex until lastIndex) {
            particleGroupBuffer[i] = group
        }

        updateContacts(true)
        if (groupDef.flags and pairFlags != 0) {
            for (k in 0 until contactCount) {
                val contact = contactBuffer[k]
                var a = contact.indexA
                var b = contact.indexB
                if (a > b) {
                    val temp = a
                    a = b
                    b = temp
                }
                if (firstIndex <= a && b < lastIndex) {
                    if (pairCount >= pairCapacity) {
                        val oldCapacity = pairCapacity
                        val newCapacity = if (pairCount != 0) 2 * pairCount else Settings.minParticleBufferCapacity
                        pairBuffer = BufferUtils.reallocateBuffer({ Pair() }, pairBuffer, oldCapacity, newCapacity)
                        pairCapacity = newCapacity
                    }
                    val pair = pairBuffer[pairCount]
                    pair.indexA = a
                    pair.indexB = b
                    pair.flags = contact.flags
                    pair.strength = groupDef.strength
                    pair.distance = MathUtils.distance(positionBuffer.data!![a], positionBuffer.data!![b])
                    pairCount++
                }
            }
        }
        if (groupDef.flags and triadFlags != 0) {
            val diagram = VoronoiDiagram(lastIndex - firstIndex)
            for (i in firstIndex until lastIndex) {
                diagram.addGenerator(positionBuffer.data!![i], i)
            }
            diagram.generate(stride / 2)
            createParticleGroupCallback.system = this
            createParticleGroupCallback.def = groupDef
            createParticleGroupCallback.firstIndex = firstIndex
            diagram.getNodes(createParticleGroupCallback)
        }
        if (groupDef.groupFlags and ParticleGroupType.solidParticleGroup != 0) {
            computeDepthForGroup(group)
        }

        return group
    }

    fun joinParticleGroups(groupA: ParticleGroup, groupB: ParticleGroup) {
        assert(groupA != groupB)
        RotateBuffer(groupB.firstIndex, groupB.lastIndex, particleCount)
        assert(groupB.lastIndex == particleCount)
        RotateBuffer(groupA.firstIndex, groupA.lastIndex, groupB.firstIndex)
        assert(groupA.lastIndex == groupB.firstIndex)

        var particleFlags = 0
        for (i in groupA.firstIndex until groupB.lastIndex) {
            particleFlags = particleFlags or flagsBuffer.data!![i]
        }

        updateContacts(true)
        if (particleFlags and pairFlags != 0) {
            for (k in 0 until contactCount) {
                val contact = contactBuffer[k]
                var a = contact.indexA
                var b = contact.indexB
                if (a > b) {
                    val temp = a
                    a = b
                    b = temp
                }
                if (groupA.firstIndex <= a && a < groupA.lastIndex && groupB.firstIndex <= b
                    && b < groupB.lastIndex
                ) {
                    if (pairCount >= pairCapacity) {
                        val oldCapacity = pairCapacity
                        val newCapacity = if (pairCount != 0) 2 * pairCount else Settings.minParticleBufferCapacity
                        pairBuffer = BufferUtils.reallocateBuffer({ Pair() }, pairBuffer, oldCapacity, newCapacity)
                        pairCapacity = newCapacity
                    }
                    val pair = pairBuffer[pairCount]
                    pair.indexA = a
                    pair.indexB = b
                    pair.flags = contact.flags
                    pair.strength = MathUtils.min(groupA.strength, groupB.strength)
                    pair.distance = MathUtils.distance(positionBuffer.data!![a], positionBuffer.data!![b])
                    pairCount++
                }
            }
        }
        if (particleFlags and triadFlags != 0) {
            val diagram = VoronoiDiagram(groupB.lastIndex - groupA.firstIndex)
            for (i in groupA.firstIndex until groupB.lastIndex) {
                if (flagsBuffer.data!![i] and ParticleType.zombieParticle == 0) {
                    diagram.addGenerator(positionBuffer.data!![i], i)
                }
            }
            diagram.generate(particleStride / 2)
            val callback = JoinParticleGroupsCallback()
            callback.system = this
            callback.groupA = groupA
            callback.groupB = groupB
            diagram.getNodes(callback)
        }

        for (i in groupB.firstIndex until groupB.lastIndex) {
            particleGroupBuffer[i] = groupA
        }
        val groupFlags = groupA.groupFlags or groupB.groupFlags
        groupA.groupFlags = groupFlags
        groupA.lastIndex = groupB.lastIndex
        groupB.firstIndex = groupB.lastIndex
        destroyParticleGroup(groupB)

        if (groupFlags and ParticleGroupType.solidParticleGroup != 0) {
            computeDepthForGroup(groupA)
        }
    }

    // Only called from solveZombie() or joinParticleGroups().
    internal fun destroyParticleGroup(group: ParticleGroup?) {
        assert(particleGroupCount > 0)
        assert(group != null)

        if (world.particleDestructionListener != null) {
            world.particleDestructionListener!!.sayGoodbye(group!!)
        }

        for (i in group!!.firstIndex until group.lastIndex) {
            particleGroupBuffer[i] = null
        }

        if (group.prev != null) {
            group.prev!!.next = group.next
        }
        if (group.next != null) {
            group.next!!.prev = group.prev
        }
        if (group == groupList) {
            groupList = group.next
        }

        --particleGroupCount
    }

    fun computeDepthForGroup(group: ParticleGroup) {
        for (i in group.firstIndex until group.lastIndex) {
            accumulationBuffer[i] = 0f
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            val a = contact.indexA
            val b = contact.indexB
            if (a >= group.firstIndex && a < group.lastIndex && b >= group.firstIndex
                && b < group.lastIndex
            ) {
                val w = contact.weight
                accumulationBuffer[a] += w
                accumulationBuffer[b] += w
            }
        }
        depthBuffer = requestParticleBuffer(depthBuffer)
        for (i in group.firstIndex until group.lastIndex) {
            val w = accumulationBuffer[i]
            depthBuffer!![i] = if (w < 0.8f) 0f else Float.MAX_VALUE
        }
        val interationCount = group.particleCount
        for (t in 0 until interationCount) {
            var updated = false
            for (k in 0 until contactCount) {
                val contact = contactBuffer[k]
                val a = contact.indexA
                val b = contact.indexB
                if (a >= group.firstIndex && a < group.lastIndex && b >= group.firstIndex
                    && b < group.lastIndex
                ) {
                    val r = 1 - contact.weight
                    val ap0 = depthBuffer!![a]
                    val bp0 = depthBuffer!![b]
                    val ap1 = bp0 + r
                    val bp1 = ap0 + r
                    if (ap0 > ap1) {
                        depthBuffer!![a] = ap1
                        updated = true
                    }
                    if (bp0 > bp1) {
                        depthBuffer!![b] = bp1
                        updated = true
                    }
                }
            }
            if (!updated) break
        }
        for (i in group.firstIndex until group.lastIndex) {
            val p = depthBuffer!![i]
            if (p < Float.MAX_VALUE) {
                depthBuffer!![i] *= particleDiameter
            } else {
                depthBuffer!![i] = 0f
            }
        }
    }

    fun addContact(a: Int, b: Int) {
        assert(a != b)
        val pa = positionBuffer.data!![a]
        val pb = positionBuffer.data!![b]
        val dx = pb.x - pa.x
        val dy = pb.y - pa.y
        val d2 = dx * dx + dy * dy
        //    assert(d2 != 0);
        if (d2 < squaredDiameter) {
            if (contactCount >= contactCapacity) {
                val oldCapacity = contactCapacity
                val newCapacity = if (contactCount != 0) 2 * contactCount else Settings.minParticleBufferCapacity
                contactBuffer = BufferUtils.reallocateBuffer(
                    { ParticleContact() }, contactBuffer, oldCapacity,
                    newCapacity
                )
                contactCapacity = newCapacity
            }
            val invD = if (d2 != 0f) MathUtils.sqrt(1 / d2) else Float.MAX_VALUE
            val contact = contactBuffer[contactCount]
            contact.indexA = a
            contact.indexB = b
            contact.flags = flagsBuffer.data!![a] or flagsBuffer.data!![b]
            contact.weight = 1 - d2 * invD * inverseDiameter
            contact.normal.x = invD * dx
            contact.normal.y = invD * dy
            contactCount++
        }
    }

    fun updateContacts(exceptZombie: Boolean) {
        for (p in 0 until proxyCount) {
            val proxy = proxyBuffer[p]
            val i = proxy.index
            val pos = positionBuffer.data!![i]
            proxy.tag = computeTag(inverseDiameter * pos.x, inverseDiameter * pos.y)
        }
        Arrays_sort(proxyBuffer, 0, proxyCount)
        contactCount = 0
        var c_index = 0
        for (i in 0 until proxyCount) {
            val a = proxyBuffer[i]
            val rightTag = computeRelativeTag(a.tag, 1, 0)
            for (j in i + 1 until proxyCount) {
                val b = proxyBuffer[j]
                if (rightTag < b.tag) {
                    break
                }
                addContact(a.index, b.index)
            }
            val bottomLeftTag = computeRelativeTag(a.tag, -1, 1)
            while (c_index < proxyCount) {
                val c = proxyBuffer[c_index]
                if (bottomLeftTag <= c.tag) {
                    break
                }
                c_index++
            }
            val bottomRightTag = computeRelativeTag(a.tag, 1, 1)

            for (b_index in c_index until proxyCount) {
                val b = proxyBuffer[b_index]
                if (bottomRightTag < b.tag) {
                    break
                }
                addContact(a.index, b.index)
            }
        }
        if (exceptZombie) {
            var j = contactCount
            var i = 0
            while (i < j) {
                if (contactBuffer[i].flags and ParticleType.zombieParticle != 0) {
                    --j
                    val temp = contactBuffer[j]
                    contactBuffer[j] = contactBuffer[i]
                    contactBuffer[i] = temp
                    --i
                }
                i++
            }
            contactCount = j
        }
    }

    fun updateBodyContacts() {
        val aabb = temp
        aabb.lowerBound.x = Float.MAX_VALUE
        aabb.lowerBound.y = Float.MAX_VALUE
        aabb.upperBound.x = -Float.MAX_VALUE
        aabb.upperBound.y = -Float.MAX_VALUE
        for (i in 0 until particleCount) {
            val p = positionBuffer.data!![i]
            Vec2.minToOut(aabb.lowerBound, p, aabb.lowerBound)
            Vec2.maxToOut(aabb.upperBound, p, aabb.upperBound)
        }
        aabb.lowerBound.x -= particleDiameter
        aabb.lowerBound.y -= particleDiameter
        aabb.upperBound.x += particleDiameter
        aabb.upperBound.y += particleDiameter
        bodyContactCount = 0

        ubccallback.system = this
        world.queryAABB(ubccallback, aabb)
    }

    fun solveCollision(step: TimeStep) {
        val aabb = temp
        val lowerBound = aabb.lowerBound
        val upperBound = aabb.upperBound
        lowerBound.x = Float.MAX_VALUE
        lowerBound.y = Float.MAX_VALUE
        upperBound.x = -Float.MAX_VALUE
        upperBound.y = -Float.MAX_VALUE
        for (i in 0 until particleCount) {
            val v = velocityBuffer.data!![i]
            val p1 = positionBuffer.data!![i]
            val p1x = p1.x
            val p1y = p1.y
            val p2x = p1x + step.dt * v.x
            val p2y = p1y + step.dt * v.y
            val bx = if (p1x < p2x) p1x else p2x
            val by = if (p1y < p2y) p1y else p2y
            lowerBound.x = if (lowerBound.x < bx) lowerBound.x else bx
            lowerBound.y = if (lowerBound.y < by) lowerBound.y else by
            val b1x = if (p1x > p2x) p1x else p2x
            val b1y = if (p1y > p2y) p1y else p2y
            upperBound.x = if (upperBound.x > b1x) upperBound.x else b1x
            upperBound.y = if (upperBound.y > b1y) upperBound.y else b1y
        }
        sccallback.step = step
        sccallback.system = this
        world.queryAABB(sccallback, aabb)
    }

    fun solve(step: TimeStep) {
        ++timestamp
        if (particleCount == 0) return
        allParticleFlags = 0
        for (i in 0 until particleCount) {
            allParticleFlags = allParticleFlags or flagsBuffer.data!![i]
        }
        if (allParticleFlags and ParticleType.zombieParticle != 0) {
            solveZombie()
        }
        if (particleCount == 0) return
        allGroupFlags = 0
        var group = groupList
        while (group != null) {
            allGroupFlags = allGroupFlags or group.groupFlags
            group = group.next
        }
        val gravityx = step.dt * particleGravityScale * world.gravity.x
        val gravityy = step.dt * particleGravityScale * world.gravity.y
        val criticalVelocytySquared = getCriticalVelocitySquared(step)
        for (i in 0 until particleCount) {
            val v = velocityBuffer.data!![i]
            v.x += gravityx
            v.y += gravityy
            val v2 = v.x * v.x + v.y * v.y
            if (v2 > criticalVelocytySquared) {
                val a = if (v2 == 0f) Float.MAX_VALUE else MathUtils.sqrt(criticalVelocytySquared / v2)
                v.x *= a
                v.y *= a
            }
        }
        solveCollision(step)
        if (allGroupFlags and ParticleGroupType.rigidParticleGroup != 0) {
            solveRigid(step)
        }
        if (allParticleFlags and ParticleType.wallParticle != 0) {
            solveWall(step)
        }
        for (i in 0 until particleCount) {
            val pos = positionBuffer.data!![i]
            val vel = velocityBuffer.data!![i]
            pos.x += step.dt * vel.x
            pos.y += step.dt * vel.y
        }
        updateBodyContacts()
        updateContacts(false)
        if (allParticleFlags and ParticleType.viscousParticle != 0) {
            solveViscous(step)
        }
        if (allParticleFlags and ParticleType.powderParticle != 0) {
            solvePowder(step)
        }
        if (allParticleFlags and ParticleType.tensileParticle != 0) {
            solveTensile(step)
        }
        if (allParticleFlags and ParticleType.elasticParticle != 0) {
            solveElastic(step)
        }
        if (allParticleFlags and ParticleType.springParticle != 0) {
            solveSpring(step)
        }
        if (allGroupFlags and ParticleGroupType.solidParticleGroup != 0) {
            solveSolid(step)
        }
        if (allParticleFlags and ParticleType.colorMixingParticle != 0) {
            solveColorMixing(step)
        }
        solvePressure(step)
        solveDamping(step)
    }

    internal fun solvePressure(step: TimeStep) {
        // calculates the sum of contact-weights for each particle
        // that means dimensionless density
        for (i in 0 until particleCount) {
            accumulationBuffer[i] = 0f
        }
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer[k]
            val a = contact.index
            val w = contact.weight
            accumulationBuffer[a] += w
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            val a = contact.indexA
            val b = contact.indexB
            val w = contact.weight
            accumulationBuffer[a] += w
            accumulationBuffer[b] += w
        }
        // ignores powder particles
        if (allParticleFlags and noPressureFlags != 0) {
            for (i in 0 until particleCount) {
                if (flagsBuffer.data!![i] and noPressureFlags != 0) {
                    accumulationBuffer[i] = 0f
                }
            }
        }
        // calculates pressure as a linear function of density
        val pressurePerWeight = pressureStrength * getCriticalPressure(step)
        for (i in 0 until particleCount) {
            val w = accumulationBuffer[i]
            val h = pressurePerWeight * MathUtils.max(
                0.0f,
                MathUtils.min(w, Settings.maxParticleWeight) - Settings.minParticleWeight
            )
            accumulationBuffer[i] = h
        }
        // applies pressure between each particles in contact
        val velocityPerPressure = step.dt / (density * particleDiameter)
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer[k]
            val a = contact.index
            val b = contact.body
            val w = contact.weight
            val m = contact.mass
            val n = contact.normal
            val p = positionBuffer.data!![a]
            val h = accumulationBuffer[a] + pressurePerWeight * w
            val f = tempVec
            val coef = velocityPerPressure * w * m * h
            f.x = coef * n.x
            f.y = coef * n.y
            val velData = velocityBuffer.data!![a]
            val particleInvMass = particleInvMass
            velData.x -= particleInvMass * f.x
            velData.y -= particleInvMass * f.y
            b!!.applyLinearImpulse(f, p, true)
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            val a = contact.indexA
            val b = contact.indexB
            val w = contact.weight
            val n = contact.normal
            val h = accumulationBuffer[a] + accumulationBuffer[b]
            val fx = velocityPerPressure * w * h * n.x
            val fy = velocityPerPressure * w * h * n.y
            val velDataA = velocityBuffer.data!![a]
            val velDataB = velocityBuffer.data!![b]
            velDataA.x -= fx
            velDataA.y -= fy
            velDataB.x += fx
            velDataB.y += fy
        }
    }

    internal fun solveDamping(step: TimeStep) {
        // reduces normal velocity of each contact
        val damping = particleDamping
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer[k]
            val a = contact.index
            val b = contact.body
            val w = contact.weight
            val m = contact.mass
            val n = contact.normal
            val p = positionBuffer.data!![a]
            val tempX = p.x - b!!.sweep.c.x
            val tempY = p.y - b.sweep.c.y
            val velA = velocityBuffer.data!![a]
            // getLinearVelocityFromWorldPointToOut, with -= velA
            val vx = -b._angularVelocity * tempY + b._linearVelocity.x - velA.x
            val vy = b._angularVelocity * tempX + b._linearVelocity.y - velA.y
            // done
            val vn = vx * n.x + vy * n.y
            if (vn < 0) {
                val f = tempVec
                f.x = damping * w * m * vn * n.x
                f.y = damping * w * m * vn * n.y
                val invMass = particleInvMass
                velA.x += invMass * f.x
                velA.y += invMass * f.y
                f.x = -f.x
                f.y = -f.y
                b.applyLinearImpulse(f, p, true)
            }
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            val a = contact.indexA
            val b = contact.indexB
            val w = contact.weight
            val n = contact.normal
            val velA = velocityBuffer.data!![a]
            val velB = velocityBuffer.data!![b]
            val vx = velB.x - velA.x
            val vy = velB.y - velA.y
            val vn = vx * n.x + vy * n.y
            if (vn < 0) {
                val fx = damping * w * vn * n.x
                val fy = damping * w * vn * n.y
                velA.x += fx
                velA.y += fy
                velB.x -= fx
                velB.y -= fy
            }
        }
    }

    fun solveWall(step: TimeStep) {
        for (i in 0 until particleCount) {
            if (flagsBuffer.data!![i] and ParticleType.wallParticle != 0) {
                val r = velocityBuffer.data!![i]
                r.x = 0.0f
                r.y = 0.0f
            }
        }
    }

    internal fun solveRigid(step: TimeStep) {
        var group = groupList
        while (group != null) {
            if (group.groupFlags and ParticleGroupType.rigidParticleGroup != 0) {
                group.updateStatistics()
                val temp = tempVec
                val cross = tempVec2
                val rotation = tempRot
                rotation.setRadians(step.dt * group._angularVelocity)
                Rot.mulToOutUnsafe(rotation, group._center, cross)
                temp.set(group._linearVelocity).mulLocal(step.dt).addLocal(group._center).subLocal(cross)
                tempXf.p.set(temp)
                tempXf.q.set(rotation)
                Transform.mulToOut(tempXf, group.transform, group.transform)
                val velocityTransform = tempXf2
                velocityTransform.p.x = step.invDt * tempXf.p.x
                velocityTransform.p.y = step.invDt * tempXf.p.y
                velocityTransform.q.s = step.invDt * tempXf.q.s
                velocityTransform.q.c = step.invDt * (tempXf.q.c - 1)
                for (i in group.firstIndex until group.lastIndex) {
                    Transform.mulToOutUnsafe(
                        velocityTransform,
                        positionBuffer.data!![i],
                        velocityBuffer.data!![i]
                    )
                }
            }
            group = group.next
        }
    }

    internal fun solveElastic(step: TimeStep) {
        val elasticStrength = step.invDt * elasticStrength
        for (k in 0 until triadCount) {
            val triad = triadBuffer[k]
            if (triad.flags and ParticleType.elasticParticle != 0) {
                val a = triad.indexA
                val b = triad.indexB
                val c = triad.indexC
                val oa = triad.pa
                val ob = triad.pb
                val oc = triad.pc
                val pa = positionBuffer.data!![a]
                val pb = positionBuffer.data!![b]
                val pc = positionBuffer.data!![c]
                val px = 1f / 3 * (pa.x + pb.x + pc.x)
                val py = 1f / 3 * (pa.y + pb.y + pc.y)
                var rs = Vec2.cross(oa, pa) + Vec2.cross(ob, pb) + Vec2.cross(oc, pc)
                var rc = Vec2.dot(oa, pa) + Vec2.dot(ob, pb) + Vec2.dot(oc, pc)
                val r2 = rs * rs + rc * rc
                val invR = if (r2 == 0f) Float.MAX_VALUE else MathUtils.sqrt(1f / r2)
                rs *= invR
                rc *= invR
                val strength = elasticStrength * triad.strength
                val roax = rc * oa.x - rs * oa.y
                val roay = rs * oa.x + rc * oa.y
                val robx = rc * ob.x - rs * ob.y
                val roby = rs * ob.x + rc * ob.y
                val rocx = rc * oc.x - rs * oc.y
                val rocy = rs * oc.x + rc * oc.y
                val va = velocityBuffer.data!![a]
                val vb = velocityBuffer.data!![b]
                val vc = velocityBuffer.data!![c]
                va.x += strength * (roax - (pa.x - px))
                va.y += strength * (roay - (pa.y - py))
                vb.x += strength * (robx - (pb.x - px))
                vb.y += strength * (roby - (pb.y - py))
                vc.x += strength * (rocx - (pc.x - px))
                vc.y += strength * (rocy - (pc.y - py))
            }
        }
    }

    internal fun solveSpring(step: TimeStep) {
        val springStrength = step.invDt * springStrength
        for (k in 0 until pairCount) {
            val pair = pairBuffer[k]
            if (pair.flags and ParticleType.springParticle != 0) {
                val a = pair.indexA
                val b = pair.indexB
                val pa = positionBuffer.data!![a]
                val pb = positionBuffer.data!![b]
                val dx = pb.x - pa.x
                val dy = pb.y - pa.y
                val r0 = pair.distance
                var r1 = MathUtils.sqrt(dx * dx + dy * dy)
                if (r1 == 0f) r1 = Float.MAX_VALUE
                val strength = springStrength * pair.strength
                val fx = strength * (r0 - r1) / r1 * dx
                val fy = strength * (r0 - r1) / r1 * dy
                val va = velocityBuffer.data!![a]
                val vb = velocityBuffer.data!![b]
                va.x -= fx
                va.y -= fy
                vb.x += fx
                vb.y += fy
            }
        }
    }

    internal fun solveTensile(step: TimeStep) {
        accumulation2Buffer = requestParticleBuffer({ Vec2() }, accumulation2Buffer)
        for (i in 0 until particleCount) {
            accumulationBuffer[i] = 0f
            accumulation2Buffer[i].setZero()
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            if (contact.flags and ParticleType.tensileParticle != 0) {
                val a = contact.indexA
                val b = contact.indexB
                val w = contact.weight
                val n = contact.normal
                accumulationBuffer[a] += w
                accumulationBuffer[b] += w
                val a2A = accumulation2Buffer[a]
                val a2B = accumulation2Buffer[b]
                val inter = (1 - w) * w
                a2A.x -= inter * n.x
                a2A.y -= inter * n.y
                a2B.x += inter * n.x
                a2B.y += inter * n.y
            }
        }
        val strengthA = surfaceTensionStrengthA * getCriticalVelocity(step)
        val strengthB = surfaceTensionStrengthB * getCriticalVelocity(step)
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            if (contact.flags and ParticleType.tensileParticle != 0) {
                val a = contact.indexA
                val b = contact.indexB
                val w = contact.weight
                val n = contact.normal
                val a2A = accumulation2Buffer[a]
                val a2B = accumulation2Buffer[b]
                val h = accumulationBuffer[a] + accumulationBuffer[b]
                val sx = a2B.x - a2A.x
                val sy = a2B.y - a2A.y
                val fn = (strengthA * (h - 2) + strengthB * (sx * n.x + sy * n.y)) * w
                val fx = fn * n.x
                val fy = fn * n.y
                val va = velocityBuffer.data!![a]
                val vb = velocityBuffer.data!![b]
                va.x -= fx
                va.y -= fy
                vb.x += fx
                vb.y += fy
            }
        }
    }

    internal fun solveViscous(step: TimeStep) {
        val viscousStrength = viscousStrength
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer[k]
            val a = contact.index
            if (flagsBuffer.data!![a] and ParticleType.viscousParticle != 0) {
                val b = contact.body
                val w = contact.weight
                val m = contact.mass
                val p = positionBuffer.data!![a]
                val va = velocityBuffer.data!![a]
                val tempX = p.x - b!!.sweep.c.x
                val tempY = p.y - b.sweep.c.y
                val vx = -b._angularVelocity * tempY + b._linearVelocity.x - va.x
                val vy = b._angularVelocity * tempX + b._linearVelocity.y - va.y
                val f = tempVec
                val pInvMass = particleInvMass
                f.x = viscousStrength * m * w * vx
                f.y = viscousStrength * m * w * vy
                va.x += pInvMass * f.x
                va.y += pInvMass * f.y
                f.x = -f.x
                f.y = -f.y
                b.applyLinearImpulse(f, p, true)
            }
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            if (contact.flags and ParticleType.viscousParticle != 0) {
                val a = contact.indexA
                val b = contact.indexB
                val w = contact.weight
                val va = velocityBuffer.data!![a]
                val vb = velocityBuffer.data!![b]
                val vx = vb.x - va.x
                val vy = vb.y - va.y
                val fx = viscousStrength * w * vx
                val fy = viscousStrength * w * vy
                va.x += fx
                va.y += fy
                vb.x -= fx
                vb.y -= fy
            }
        }
    }

    internal fun solvePowder(step: TimeStep) {
        val powderStrength = powderStrength * getCriticalVelocity(step)
        val minWeight = 1.0f - Settings.particleStride
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer[k]
            val a = contact.index
            if (flagsBuffer.data!![a] and ParticleType.powderParticle != 0) {
                val w = contact.weight
                if (w > minWeight) {
                    val b = contact.body
                    val m = contact.mass
                    val p = positionBuffer.data!![a]
                    val n = contact.normal
                    val f = tempVec
                    val va = velocityBuffer.data!![a]
                    val inter = powderStrength * m * (w - minWeight)
                    val pInvMass = particleInvMass
                    f.x = inter * n.x
                    f.y = inter * n.y
                    va.x -= pInvMass * f.x
                    va.y -= pInvMass * f.y
                    b!!.applyLinearImpulse(f, p, true)
                }
            }
        }
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            if (contact.flags and ParticleType.powderParticle != 0) {
                val w = contact.weight
                if (w > minWeight) {
                    val a = contact.indexA
                    val b = contact.indexB
                    val n = contact.normal
                    val va = velocityBuffer.data!![a]
                    val vb = velocityBuffer.data!![b]
                    val inter = powderStrength * (w - minWeight)
                    val fx = inter * n.x
                    val fy = inter * n.y
                    va.x -= fx
                    va.y -= fy
                    vb.x += fx
                    vb.y += fy
                }
            }
        }
    }

    internal fun solveSolid(step: TimeStep) {
        // applies extra repulsive force from solid particle groups
        depthBuffer = requestParticleBuffer(depthBuffer)
        val ejectionStrength = step.invDt * ejectionStrength
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            val a = contact.indexA
            val b = contact.indexB
            if (particleGroupBuffer[a] != particleGroupBuffer[b]) {
                val w = contact.weight
                val n = contact.normal
                val h = depthBuffer!![a] + depthBuffer!![b]
                val va = velocityBuffer.data!![a]
                val vb = velocityBuffer.data!![b]
                val inter = ejectionStrength * h * w
                val fx = inter * n.x
                val fy = inter * n.y
                va.x -= fx
                va.y -= fy
                vb.x += fx
                vb.y += fy
            }
        }
    }

    internal fun solveColorMixing(step: TimeStep) {
        // mixes color between contacting particles
        colorBuffer.data = requestParticleBuffer({ ParticleColor() }, colorBuffer.data)
        val colorMixing256 = (256 * colorMixingStrength).toInt()
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            val a = contact.indexA
            val b = contact.indexB
            if (flagsBuffer.data!![a] and flagsBuffer.data!![b] and ParticleType.colorMixingParticle != 0) {
                val colorA = colorBuffer.data!![a]
                val colorB = colorBuffer.data!![b]
                val dr = (colorMixing256 * ((colorB.r.toInt() and 0xFF) - (colorA.r.toInt() and 0xFF))) ushr 8
                val dg = (colorMixing256 * ((colorB.g.toInt() and 0xFF) - (colorA.g.toInt() and 0xFF))) ushr 8
                val db = (colorMixing256 * ((colorB.b.toInt() and 0xFF) - (colorA.b.toInt() and 0xFF))) ushr 8
                val da = (colorMixing256 * ((colorB.a.toInt() and 0xFF) - (colorA.a.toInt() and 0xFF))) ushr 8
                colorA.r = (colorA.r.toInt() + dr).toByte()
                colorA.g = (colorA.g.toInt() + dg).toByte()
                colorA.b = (colorA.b.toInt() + db).toByte()
                colorA.a = (colorA.a.toInt() + da).toByte()
                colorB.r = (colorB.r.toInt() - dr).toByte()
                colorB.g = (colorB.g.toInt() - dg).toByte()
                colorB.b = (colorB.b.toInt() - db).toByte()
                colorB.a = (colorB.a.toInt() - da).toByte()
            }
        }
    }

    internal fun solveZombie() {
        // removes particles with zombie flag
        var newCount = 0
        val newIndices = IntArray(particleCount)
        for (i in 0 until particleCount) {
            val flags = flagsBuffer.data!![i]
            if (flags and ParticleType.zombieParticle != 0) {
                val destructionListener = world.particleDestructionListener
                if (flags and ParticleType.destructionListener != 0 && destructionListener != null) {
                    destructionListener.sayGoodbye(i)
                }
                newIndices[i] = Settings.invalidParticleIndex
            } else {
                newIndices[i] = newCount
                if (i != newCount) {
                    flagsBuffer.data!![newCount] = flagsBuffer.data!![i]
                    positionBuffer.data!![newCount].set(positionBuffer.data!![i])
                    velocityBuffer.data!![newCount].set(velocityBuffer.data!![i])
                    particleGroupBuffer.set(newCount, particleGroupBuffer[i]!!)
                    if (depthBuffer != null) {
                        depthBuffer!![newCount] = depthBuffer!![i]
                    }
                    if (colorBuffer.data != null) {
                        colorBuffer.data!![newCount].set(colorBuffer.data!![i])
                    }
                    if (userDataBuffer.data != null) {
                        userDataBuffer!!.data!![newCount] = userDataBuffer.data!![i]
                    }
                }
                newCount++
            }
        }

        // update proxies
        for (k in 0 until proxyCount) {
            val proxy = proxyBuffer[k]
            proxy.index = newIndices[proxy.index]
        }

        // Proxy lastProxy = std.remove_if(
        // m_proxyBuffer, m_proxyBuffer + m_proxyCount,
        // Test.IsProxyInvalid);
        // m_proxyCount = (int) (lastProxy - m_proxyBuffer);
        var j = proxyCount
        run {
            var i = 0
            while (i < j) {
                if (Test.IsProxyInvalid(proxyBuffer[i])) {
                    --j
                    val temp = proxyBuffer[j]
                    proxyBuffer[j] = proxyBuffer[i]
                    proxyBuffer[i] = temp
                    --i
                }
                i++
            }
        }
        proxyCount = j

        // update contacts
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            contact.indexA = newIndices[contact.indexA]
            contact.indexB = newIndices[contact.indexB]
        }
        // ParticleContact lastContact = std.remove_if(
        // m_contactBuffer, m_contactBuffer + m_contactCount,
        // Test.IsContactInvalid);
        // m_contactCount = (int) (lastContact - m_contactBuffer);
        j = contactCount
        run {
            var i = 0
            while (i < j) {
                if (Test.IsContactInvalid(contactBuffer[i])) {
                    --j
                    val temp = contactBuffer[j]
                    contactBuffer[j] = contactBuffer[i]
                    contactBuffer[i] = temp
                    --i
                }
                i++
            }
        }
        contactCount = j

        // update particle-body contacts
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer[k]
            contact.index = newIndices[contact.index]
        }
        // ParticleBodyContact lastBodyContact = std.remove_if(
        // m_bodyContactBuffer, m_bodyContactBuffer + m_bodyContactCount,
        // Test.IsBodyContactInvalid);
        // m_bodyContactCount = (int) (lastBodyContact - m_bodyContactBuffer);
        j = bodyContactCount
        run {
            var i = 0
            while (i < j) {
                if (Test.IsBodyContactInvalid(bodyContactBuffer[i])) {
                    --j
                    val temp = bodyContactBuffer[j]
                    bodyContactBuffer[j] = bodyContactBuffer[i]
                    bodyContactBuffer[i] = temp
                    --i
                }
                i++
            }
        }
        bodyContactCount = j

        // update pairs
        for (k in 0 until pairCount) {
            val pair = pairBuffer[k]
            pair.indexA = newIndices[pair.indexA]
            pair.indexB = newIndices[pair.indexB]
        }
        // Pair lastPair = std.remove_if(m_pairBuffer, m_pairBuffer + m_pairCount, Test.IsPairInvalid);
        // m_pairCount = (int) (lastPair - m_pairBuffer);
        j = pairCount
        run {
            var i = 0
            while (i < j) {
                if (Test.IsPairInvalid(pairBuffer[i])) {
                    --j
                    val temp = pairBuffer[j]
                    pairBuffer[j] = pairBuffer[i]
                    pairBuffer[i] = temp
                    --i
                }
                i++
            }
        }
        pairCount = j

        // update triads
        for (k in 0 until triadCount) {
            val triad = triadBuffer[k]
            triad.indexA = newIndices[triad.indexA]
            triad.indexB = newIndices[triad.indexB]
            triad.indexC = newIndices[triad.indexC]
        }
        // Triad lastTriad =
        // std.remove_if(m_triadBuffer, m_triadBuffer + m_triadCount, Test.isTriadInvalid);
        // m_triadCount = (int) (lastTriad - m_triadBuffer);
        j = triadCount
        run {
            var i = 0
            while (i < j) {
                if (Test.IsTriadInvalid(triadBuffer[i])) {
                    --j
                    val temp = triadBuffer[j]
                    triadBuffer[j] = triadBuffer[i]
                    triadBuffer[i] = temp
                    --i
                }
                i++
            }
        }
        triadCount = j

        // update groups
        run {
            var group = groupList
            while (group != null) {
                var firstIndex = newCount
                var lastIndex = 0
                var modified = false
                for (i in group!!.firstIndex until group!!.lastIndex) {
                    j = newIndices[i]
                    if (j >= 0) {
                        firstIndex = MathUtils.min(firstIndex, j)
                        lastIndex = MathUtils.max(lastIndex, j + 1)
                    } else {
                        modified = true
                    }
                }
                if (firstIndex < lastIndex) {
                    group!!.firstIndex = firstIndex
                    group!!.lastIndex = lastIndex
                    if (modified) {
                        if (group!!.groupFlags and ParticleGroupType.rigidParticleGroup != 0) {
                            group!!.toBeSplit = true
                        }
                    }
                } else {
                    group!!.firstIndex = 0
                    group!!.lastIndex = 0
                    if (group!!.destroyAutomatically) {
                        group!!.toBeDestroyed = true
                    }
                }
                group = group!!.next
            }
        }

        // update particle count
        particleCount = newCount
        // m_world.m_stackAllocator.Free(newIndices);

        // destroy bodies with no particles
        var group = groupList
        while (group != null) {
            val next = group!!.next
            if (group!!.toBeDestroyed) {
                destroyParticleGroup(group)
            } else if (group!!.toBeSplit) {
                // TODO: split the group
            }
            group = next
        }
    }

    private class NewIndices {
        internal var start: Int = 0
        internal var mid: Int = 0
        internal var end: Int = 0

        internal fun getIndex(i: Int) = when {
            i < start -> i
            i < mid -> i + end - mid
            i < end -> i + start - mid
            else -> i
        }
    }

    internal fun RotateBuffer(start: Int, mid: Int, end: Int) {
        // move the particles assigned to the given group toward the end of array
        if (start == mid || mid == end) return
        newIndices.start = start
        newIndices.mid = mid
        newIndices.end = end

        BufferUtils.rotate(flagsBuffer.data!!, start, mid, end)
        BufferUtils.rotate(positionBuffer.data!!, start, mid, end)
        BufferUtils.rotate(velocityBuffer.data!!, start, mid, end)
        BufferUtils.rotate(particleGroupBuffer as Array<ParticleGroup>, start, mid, end)
        if (depthBuffer != null) {
            BufferUtils.rotate(depthBuffer!!, start, mid, end)
        }
        if (colorBuffer.data != null) {
            BufferUtils.rotate(colorBuffer.data!!, start, mid, end)
        }
        if (userDataBuffer.data != null) {
            BufferUtils.rotate(userDataBuffer.data!!, start, mid, end)
        }

        // update proxies
        for (k in 0 until proxyCount) {
            val proxy = proxyBuffer[k]
            proxy.index = newIndices.getIndex(proxy.index)
        }

        // update contacts
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            contact.indexA = newIndices.getIndex(contact.indexA)
            contact.indexB = newIndices.getIndex(contact.indexB)
        }

        // update particle-body contacts
        for (k in 0 until bodyContactCount) {
            val contact = bodyContactBuffer[k]
            contact.index = newIndices.getIndex(contact.index)
        }

        // update pairs
        for (k in 0 until pairCount) {
            val pair = pairBuffer[k]
            pair.indexA = newIndices.getIndex(pair.indexA)
            pair.indexB = newIndices.getIndex(pair.indexB)
        }

        // update triads
        for (k in 0 until triadCount) {
            val triad = triadBuffer[k]
            triad.indexA = newIndices.getIndex(triad.indexA)
            triad.indexB = newIndices.getIndex(triad.indexB)
            triad.indexC = newIndices.getIndex(triad.indexC)
        }

        // update groups
        var group = groupList
        while (group != null) {
            group.firstIndex = newIndices.getIndex(group.firstIndex)
            group.lastIndex = newIndices.getIndex(group.lastIndex - 1) + 1
            group = group.next
        }
    }

    internal fun getCriticalVelocity(step: TimeStep): Float {
        return particleDiameter * step.invDt
    }

    internal fun getCriticalVelocitySquared(step: TimeStep): Float {
        val velocity = getCriticalVelocity(step)
        return velocity * velocity
    }

    internal fun getCriticalPressure(step: TimeStep): Float {
        return density * getCriticalVelocitySquared(step)
    }

    internal fun setParticleBuffer(buffer: ParticleBufferInt, newData: IntArray?, newCapacity: Int) {
        assert(newData != null && newCapacity != 0 || newData == null && newCapacity == 0)
        if (buffer.userSuppliedCapacity != 0) {
            // m_world.m_blockAllocator.Free(buffer.data, sizeof(T) * m_internalAllocatedCapacity);
        }
        buffer.data = newData
        buffer.userSuppliedCapacity = newCapacity
    }

    internal fun <T : Any> setParticleBuffer(buffer: ParticleBuffer<T>, newData: Array<T>?, newCapacity: Int) {
        assert(newData != null && newCapacity != 0 || newData == null && newCapacity == 0)
        if (buffer.userSuppliedCapacity != 0) {
            // m_world.m_blockAllocator.Free(buffer.data, sizeof(T) * m_internalAllocatedCapacity);
        }
        buffer.data = newData
        buffer.userSuppliedCapacity = newCapacity
    }

    fun setParticleFlagsBuffer(buffer: IntArray, capacity: Int) {
        setParticleBuffer(flagsBuffer, buffer, capacity)
    }

    fun setParticlePositionBuffer(buffer: Array<Vec2>, capacity: Int) {
        setParticleBuffer(positionBuffer, buffer, capacity)
    }

    fun setParticleVelocityBuffer(buffer: Array<Vec2>, capacity: Int) {
        setParticleBuffer(velocityBuffer, buffer, capacity)
    }

    fun setParticleColorBuffer(buffer: Array<ParticleColor>, capacity: Int) {
        setParticleBuffer(colorBuffer, buffer, capacity)
    }

    fun getParticleGroupList(): Array<ParticleGroup?>? {
        return particleGroupBuffer
    }

    fun setParticleUserDataBuffer(buffer: Array<Any>, capacity: Int) {
        setParticleBuffer(userDataBuffer, buffer, capacity)
    }

    fun queryAABB(callback: ParticleQueryCallback, aabb: AABB) {
        if (proxyCount == 0) return

        val lowerBoundX = aabb.lowerBound.x
        val lowerBoundY = aabb.lowerBound.y
        val upperBoundX = aabb.upperBound.x
        val upperBoundY = aabb.upperBound.y
        val firstProxy = lowerBound(
            proxyBuffer, proxyCount,
            computeTag(inverseDiameter * lowerBoundX, inverseDiameter * lowerBoundY)
        )
        val lastProxy = upperBound(
            proxyBuffer, proxyCount,
            computeTag(inverseDiameter * upperBoundX, inverseDiameter * upperBoundY)
        )
        for (proxy in firstProxy until lastProxy) {
            val i = proxyBuffer[proxy].index
            val p = positionBuffer.data!![i]
            if (lowerBoundX < p.x && p.x < upperBoundX && lowerBoundY < p.y && p.y < upperBoundY) {
                if (!callback.reportParticle(i)) {
                    break
                }
            }
        }
    }

    fun raycast(callback: ParticleRaycastCallback, point1: Vec2, point2: Vec2) {
        if (proxyCount == 0) return
        val firstProxy = lowerBound(
            proxyBuffer,
            proxyCount,
            computeTag(
                inverseDiameter * MathUtils.min(point1.x, point2.x) - 1,
                inverseDiameter * MathUtils.min(point1.y, point2.y) - 1
            )
        )
        val lastProxy = upperBound(
            proxyBuffer,
            proxyCount,
            computeTag(
                inverseDiameter * MathUtils.max(point1.x, point2.x) + 1,
                inverseDiameter * MathUtils.max(point1.y, point2.y) + 1
            )
        )
        var fraction = 1f
        // solving the following equation:
        // ((1-t)*point1+t*point2-position)^2=diameter^2
        // where t is a potential fraction
        val vx = point2.x - point1.x
        val vy = point2.y - point1.y
        var v2 = vx * vx + vy * vy
        if (v2 == 0f) v2 = Float.MAX_VALUE
        for (proxy in firstProxy until lastProxy) {
            val i = proxyBuffer[proxy].index
            val posI = positionBuffer.data!![i]
            val px = point1.x - posI.x
            val py = point1.y - posI.y
            val pv = px * vx + py * vy
            val p2 = px * px + py * py
            val determinant = pv * pv - v2 * (p2 - squaredDiameter)
            if (determinant >= 0) {
                val sqrtDeterminant = MathUtils.sqrt(determinant)
                // find a solution between 0 and fraction
                var t = (-pv - sqrtDeterminant) / v2
                if (t > fraction) {
                    continue
                }
                if (t < 0) {
                    t = (-pv + sqrtDeterminant) / v2
                    if (t < 0 || t > fraction) {
                        continue
                    }
                }
                val n = tempVec
                tempVec.x = px + t * vx
                tempVec.y = py + t * vy
                n.normalize()
                val point = tempVec2
                point.x = point1.x + t * vx
                point.y = point1.y + t * vy
                val f = callback.reportParticle(i, point, n, t)
                fraction = MathUtils.min(fraction, f)
                if (fraction <= 0) {
                    break
                }
            }
        }
    }

    fun computeParticleCollisionEnergy(): Float {
        var sum_v2 = 0f
        for (k in 0 until contactCount) {
            val contact = contactBuffer[k]
            val a = contact.indexA
            val b = contact.indexB
            val n = contact.normal
            val va = velocityBuffer.data!![a]
            val vb = velocityBuffer.data!![b]
            val vx = vb.x - va.x
            val vy = vb.y - va.y
            val vn = vx * n.x + vy * n.y
            if (vn < 0) {
                sum_v2 += vn * vn
            }
        }
        return 0.5f * particleMass * sum_v2
    }

    internal fun <T : Any> requestParticleBuffer(newInstance: () -> T, buffer: Array<T>?): Array<T> {
        var buffer = buffer
        if (buffer == null) {
            buffer = Array<Any>(internalAllocatedCapacity) { newInstance() } as Array<T>
        }
        return buffer
    }

    internal fun requestParticleBuffer(buffer: FloatArray?): FloatArray {
        var buffer = buffer
        if (buffer == null) {
            buffer = FloatArray(internalAllocatedCapacity)
        }
        return buffer
    }

    class ParticleBuffer<T : Any>(internal val dataClass: () -> T) {
        var data: Array<T>? = null
        internal var userSuppliedCapacity: Int = 0
    }

    internal class ParticleBufferInt {
        var data: IntArray? = null
        var userSuppliedCapacity: Int = 0
    }

    /** Used for detecting particle contacts  */
    class Proxy : Comparable<Proxy> {
        internal var index: Int = 0
        internal var tag: Long = 0

        override fun compareTo(other: Proxy): Int {
            return if (tag - other.tag < 0) -1 else if (other.tag == tag) 0 else 1
        }

        override fun equals(other: Any?): Boolean {
            if (this === other) return true
            if (other == null) return false
            if (this::class != other::class) return false
            val oth = other as Proxy?
            return tag == oth!!.tag
        }
    }

    /** Connection between two particles  */
    class Pair {
        internal var indexA: Int = 0
        internal var indexB: Int = 0
        internal var flags: Int = 0
        internal var strength: Float = 0f
        internal var distance: Float = 0f
    }

    /** Connection between three particles  */
    class Triad {
        internal var indexA: Int = 0
        internal var indexB: Int = 0
        internal var indexC: Int = 0
        internal var flags: Int = 0
        internal var strength: Float = 0f
        internal val pa = Vec2()
        internal val pb = Vec2()
        internal val pc = Vec2()
        internal var ka: Float = 0f
        internal var kb: Float = 0f
        internal var kc: Float = 0f
        internal var s: Float = 0f
    }

    // Callback used with VoronoiDiagram.
    internal class CreateParticleGroupCallback : VoronoiDiagramCallback {

        var system: ParticleSystem? = null
        var def: ParticleGroupDef? = null // pointer
        var firstIndex: Int = 0
        override fun callback(a: Int, b: Int, c: Int) {
            val pa = system!!.positionBuffer.data!![a]
            val pb = system!!.positionBuffer.data!![b]
            val pc = system!!.positionBuffer.data!![c]
            val dabx = pa.x - pb.x
            val daby = pa.y - pb.y
            val dbcx = pb.x - pc.x
            val dbcy = pb.y - pc.y
            val dcax = pc.x - pa.x
            val dcay = pc.y - pa.y
            val maxDistanceSquared = Settings.maxTriadDistanceSquared * system!!.squaredDiameter
            if (dabx * dabx + daby * daby < maxDistanceSquared
                && dbcx * dbcx + dbcy * dbcy < maxDistanceSquared
                && dcax * dcax + dcay * dcay < maxDistanceSquared
            ) {
                if (system!!.triadCount >= system!!.triadCapacity) {
                    val oldCapacity = system!!.triadCapacity
                    val newCapacity = if (system!!.triadCount != 0)
                        2 * system!!.triadCount
                    else
                        Settings.minParticleBufferCapacity
                    system!!.triadBuffer = BufferUtils.reallocateBuffer(
                        { Triad() }, system!!.triadBuffer, oldCapacity,
                        newCapacity
                    )
                    system!!.triadCapacity = newCapacity
                }
                val triad = system!!.triadBuffer[system!!.triadCount]
                triad.indexA = a
                triad.indexB = b
                triad.indexC = c
                triad.flags = (system!!.flagsBuffer.data!![a] or system!!.flagsBuffer.data!![b]
                    or system!!.flagsBuffer.data!![c])
                triad.strength = def!!.strength
                val midPointx = 1.toFloat() / 3 * (pa.x + pb.x + pc.x)
                val midPointy = 1.toFloat() / 3 * (pa.y + pb.y + pc.y)
                triad.pa.x = pa.x - midPointx
                triad.pa.y = pa.y - midPointy
                triad.pb.x = pb.x - midPointx
                triad.pb.y = pb.y - midPointy
                triad.pc.x = pc.x - midPointx
                triad.pc.y = pc.y - midPointy
                triad.ka = -(dcax * dabx + dcay * daby)
                triad.kb = -(dabx * dbcx + daby * dbcy)
                triad.kc = -(dbcx * dcax + dbcy * dcay)
                triad.s = Vec2.cross(pa, pb) + Vec2.cross(pb, pc) + Vec2.cross(pc, pa)
                system!!.triadCount++
            }
        }
    }

    // Callback used with VoronoiDiagram.
    internal class JoinParticleGroupsCallback : VoronoiDiagramCallback {

        var system: ParticleSystem? = null
        var groupA: ParticleGroup? = null
        var groupB: ParticleGroup? = null
        override fun callback(a: Int, b: Int, c: Int) {
            // Create a triad if it will contain particles from both groups.
            val countA = ((if (a < groupB!!.firstIndex) 1 else 0) + (if (b < groupB!!.firstIndex) 1 else 0)
                + if (c < groupB!!.firstIndex) 1 else 0)
            if (countA > 0 && countA < 3) {
                val af = system!!.flagsBuffer.data!![a]
                val bf = system!!.flagsBuffer.data!![b]
                val cf = system!!.flagsBuffer.data!![c]
                if (af and bf and cf and triadFlags != 0) {
                    val pa = system!!.positionBuffer.data!![a]
                    val pb = system!!.positionBuffer.data!![b]
                    val pc = system!!.positionBuffer.data!![c]
                    val dabx = pa.x - pb.x
                    val daby = pa.y - pb.y
                    val dbcx = pb.x - pc.x
                    val dbcy = pb.y - pc.y
                    val dcax = pc.x - pa.x
                    val dcay = pc.y - pa.y
                    val maxDistanceSquared = Settings.maxTriadDistanceSquared * system!!.squaredDiameter
                    if (dabx * dabx + daby * daby < maxDistanceSquared
                        && dbcx * dbcx + dbcy * dbcy < maxDistanceSquared
                        && dcax * dcax + dcay * dcay < maxDistanceSquared
                    ) {
                        if (system!!.triadCount >= system!!.triadCapacity) {
                            val oldCapacity = system!!.triadCapacity
                            val newCapacity = if (system!!.triadCount != 0)
                                2 * system!!.triadCount
                            else
                                Settings.minParticleBufferCapacity
                            system!!.triadBuffer = BufferUtils.reallocateBuffer(
                                { Triad() }, system!!.triadBuffer, oldCapacity,
                                newCapacity
                            )
                            system!!.triadCapacity = newCapacity
                        }
                        val triad = system!!.triadBuffer[system!!.triadCount]
                        triad.indexA = a
                        triad.indexB = b
                        triad.indexC = c
                        triad.flags = af or bf or cf
                        triad.strength = MathUtils.min(groupA!!.strength, groupB!!.strength)
                        val midPointx = 1.toFloat() / 3 * (pa.x + pb.x + pc.x)
                        val midPointy = 1.toFloat() / 3 * (pa.y + pb.y + pc.y)
                        triad.pa.x = pa.x - midPointx
                        triad.pa.y = pa.y - midPointy
                        triad.pb.x = pb.x - midPointx
                        triad.pb.y = pb.y - midPointy
                        triad.pc.x = pc.x - midPointx
                        triad.pc.y = pc.y - midPointy
                        triad.ka = -(dcax * dabx + dcay * daby)
                        triad.kb = -(dabx * dbcx + daby * dbcy)
                        triad.kc = -(dbcx * dcax + dbcy * dcay)
                        triad.s = Vec2.cross(pa, pb) + Vec2.cross(pb, pc) + Vec2.cross(pc, pa)
                        system!!.triadCount++
                    }
                }
            }
        }
    }

    internal class DestroyParticlesInShapeCallback : ParticleQueryCallback {
        lateinit var system: ParticleSystem
        lateinit var shape: Shape
        lateinit var xf: Transform
        var callDestructionListener: Boolean = false
        var destroyed: Int = 0

        fun init(
            system: ParticleSystem, shape: Shape, xf: Transform,
            callDestructionListener: Boolean
        ) {
            this.system = system
            this.shape = shape
            this.xf = xf
            this.destroyed = 0
            this.callDestructionListener = callDestructionListener
        }

        override fun reportParticle(index: Int): Boolean {
            assert(index >= 0 && index < system.particleCount)
            if (shape.testPoint(xf, system.positionBuffer.data!![index])) {
                system.destroyParticle(index, callDestructionListener)
                destroyed++
            }
            return true
        }
    }

    internal class UpdateBodyContactsCallback : QueryCallback {
        var system: ParticleSystem? = null

        private val tempVec = Vec2()

        override fun reportFixture(fixture: Fixture): Boolean {
            if (fixture.isSensor) {
                return true
            }
            val shape = fixture.shape
            val b = fixture.body
            val bp = b!!.worldCenter
            val bm = b.mass
            val bI = b.inertia - bm * b.localCenter.lengthSquared()
            val invBm = if (bm > 0) 1f / bm else 0f
            val invBI = if (bI > 0) 1f / bI else 0f
            val childCount = shape!!.getChildCount()
            for (childIndex in 0 until childCount) {
                val aabb = fixture.getAABB(childIndex)
                val aabblowerBoundx = aabb.lowerBound.x - system!!.particleDiameter
                val aabblowerBoundy = aabb.lowerBound.y - system!!.particleDiameter
                val aabbupperBoundx = aabb.upperBound.x + system!!.particleDiameter
                val aabbupperBoundy = aabb.upperBound.y + system!!.particleDiameter
                val firstProxy = lowerBound(
                    system!!.proxyBuffer,
                    system!!.proxyCount,
                    computeTag(system!!.inverseDiameter * aabblowerBoundx, system!!.inverseDiameter * aabblowerBoundy)
                )
                val lastProxy = upperBound(
                    system!!.proxyBuffer,
                    system!!.proxyCount,
                    computeTag(system!!.inverseDiameter * aabbupperBoundx, system!!.inverseDiameter * aabbupperBoundy)
                )

                for (proxy in firstProxy until lastProxy) {
                    val a = system!!.proxyBuffer[proxy].index
                    val ap = system!!.positionBuffer.data!![a]
                    if (aabblowerBoundx <= ap.x && ap.x <= aabbupperBoundx && aabblowerBoundy <= ap.y
                        && ap.y <= aabbupperBoundy
                    ) {
                        val d: Float
                        val n = tempVec
                        d = fixture.computeDistance(ap, childIndex, n)
                        if (d < system!!.particleDiameter) {
                            val invAm = if (system!!.flagsBuffer.data!![a] and ParticleType.wallParticle != 0)
                                0f
                            else
                                system!!.particleInvMass
                            val rpx = ap.x - bp.x
                            val rpy = ap.y - bp.y
                            val rpn = rpx * n.y - rpy * n.x
                            if (system!!.bodyContactCount >= system!!.bodyContactCapacity) {
                                val oldCapacity = system!!.bodyContactCapacity
                                val newCapacity = if (system!!.bodyContactCount != 0)
                                    2 * system!!.bodyContactCount
                                else
                                    Settings.minParticleBufferCapacity
                                system!!.bodyContactBuffer = BufferUtils.reallocateBuffer(
                                    { ParticleBodyContact() },
                                    system!!.bodyContactBuffer, oldCapacity, newCapacity
                                )
                                system!!.bodyContactCapacity = newCapacity
                            }
                            val contact = system!!.bodyContactBuffer[system!!.bodyContactCount]
                            contact.index = a
                            contact.body = b
                            contact.weight = 1 - d * system!!.inverseDiameter
                            contact.normal.x = -n.x
                            contact.normal.y = -n.y
                            contact.mass = 1 / (invAm + invBm + invBI * rpn * rpn)
                            system!!.bodyContactCount++
                        }
                    }
                }
            }
            return true
        }
    }

    internal class SolveCollisionCallback : QueryCallback {
        var system: ParticleSystem? = null
        var step: TimeStep? = null

        private val input = RayCastInput()
        private val output = RayCastOutput()
        private val tempVec = Vec2()
        private val tempVec2 = Vec2()

        override fun reportFixture(fixture: Fixture): Boolean {
            if (fixture.isSensor) return true
            val shape = fixture.shape
            val body = fixture.body
            val childCount = shape!!.getChildCount()
            for (childIndex in 0 until childCount) {
                val aabb = fixture.getAABB(childIndex)
                val aabblowerBoundx = aabb.lowerBound.x - system!!.particleDiameter
                val aabblowerBoundy = aabb.lowerBound.y - system!!.particleDiameter
                val aabbupperBoundx = aabb.upperBound.x + system!!.particleDiameter
                val aabbupperBoundy = aabb.upperBound.y + system!!.particleDiameter
                val firstProxy = lowerBound(
                    system!!.proxyBuffer,
                    system!!.proxyCount,
                    computeTag(system!!.inverseDiameter * aabblowerBoundx, system!!.inverseDiameter * aabblowerBoundy)
                )
                val lastProxy = upperBound(
                    system!!.proxyBuffer,
                    system!!.proxyCount,
                    computeTag(system!!.inverseDiameter * aabbupperBoundx, system!!.inverseDiameter * aabbupperBoundy)
                )

                for (proxy in firstProxy until lastProxy) {
                    val a = system!!.proxyBuffer[proxy].index
                    val ap = system!!.positionBuffer.data!![a]
                    if (aabblowerBoundx <= ap.x && ap.x <= aabbupperBoundx && aabblowerBoundy <= ap.y
                        && ap.y <= aabbupperBoundy
                    ) {
                        val av = system!!.velocityBuffer.data!![a]
                        val temp = tempVec
                        Transform.mulTransToOutUnsafe(body!!.xf0, ap, temp)
                        Transform.mulToOutUnsafe(body.xf, temp, input.p1)
                        input.p2.x = ap.x + step!!.dt * av.x
                        input.p2.y = ap.y + step!!.dt * av.y
                        input.maxFraction = 1f
                        if (fixture.raycast(output, input, childIndex)) {
                            val p = tempVec
                            p.x = ((1 - output.fraction) * input.p1.x + output.fraction * input.p2.x
                                + Settings.linearSlop * output.normal.x)
                            p.y = ((1 - output.fraction) * input.p1.y + output.fraction * input.p2.y
                                + Settings.linearSlop * output.normal.y)

                            val vx = step!!.invDt * (p.x - ap.x)
                            val vy = step!!.invDt * (p.y - ap.y)
                            av.x = vx
                            av.y = vy
                            val particleMass = system!!.particleMass
                            val ax = particleMass * (av.x - vx)
                            val ay = particleMass * (av.y - vy)
                            val b = output.normal
                            val fdn = ax * b.x + ay * b.y
                            val f = tempVec2
                            f.x = fdn * b.x
                            f.y = fdn * b.y
                            body.applyLinearImpulse(f, p, true)
                        }
                    }
                }
            }
            return true
        }
    }

    internal object Test {
        fun IsProxyInvalid(proxy: Proxy): Boolean {
            return proxy.index < 0
        }

        fun IsContactInvalid(contact: ParticleContact): Boolean {
            return contact.indexA < 0 || contact.indexB < 0
        }

        fun IsBodyContactInvalid(contact: ParticleBodyContact): Boolean {
            return contact.index < 0
        }

        fun IsPairInvalid(pair: Pair): Boolean {
            return pair.indexA < 0 || pair.indexB < 0
        }

        fun IsTriadInvalid(triad: Triad): Boolean {
            return triad.indexA < 0 || triad.indexB < 0 || triad.indexC < 0
        }
    }

    companion object {
        /** All particle types that require creating pairs  */
        private val pairFlags = ParticleType.springParticle

        /** All particle types that require creating triads  */
        private val triadFlags = ParticleType.elasticParticle

        /** All particle types that require computing depth  */
        private val noPressureFlags = ParticleType.powderParticle

        internal val xTruncBits = 12
        internal val yTruncBits = 12
        internal val tagBits = 8 * 4 - 1  /* sizeof(int) */
        internal val yOffset = (1 shl yTruncBits - 1).toLong()
        internal val yShift = tagBits - yTruncBits
        internal val xShift = tagBits - yTruncBits - xTruncBits
        internal val xScale = (1 shl xShift).toLong()
        internal val xOffset = xScale * (1 shl xTruncBits - 1)
        internal val xMask = (1 shl xTruncBits) - 1
        internal val yMask = (1 shl yTruncBits) - 1

        internal fun computeTag(x: Float, y: Float): Long {
            return ((y + yOffset).toLong() shl yShift) + ((xScale * x).toLong() + xOffset)
        }

        internal fun computeRelativeTag(tag: Long, x: Int, y: Int): Long {
            return tag + (y shl yShift).toLong() + (x shl xShift).toLong()
        }

        internal fun limitCapacity(capacity: Int, maxCount: Int): Int {
            return if (maxCount != 0 && capacity > maxCount) maxCount else capacity
        }

        private fun lowerBound(ray: Array<Proxy>, length: Int, tag: Long): Int {
            var length = length
            var left = 0
            var step: Int
            var curr: Int
            while (length > 0) {
                step = length / 2
                curr = left + step
                if (ray[curr].tag < tag) {
                    left = curr + 1
                    length -= step + 1
                } else {
                    length = step
                }
            }
            return left
        }

        private fun upperBound(ray: Array<Proxy>, length: Int, tag: Long): Int {
            var length = length
            var left = 0
            var step: Int
            var curr: Int
            while (length > 0) {
                step = length / 2
                curr = left + step
                if (ray[curr].tag <= tag) {
                    left = curr + 1
                    length -= step + 1
                } else {
                    length = step
                }
            }
            return left
        }

        // reallocate a buffer
        internal fun <T : Any> reallocateBuffer(
            buffer: ParticleBuffer<T>, oldCapacity: Int, newCapacity: Int,
            deferred: Boolean
        ): Array<T>? {
            assert(newCapacity > oldCapacity)
            return BufferUtils.reallocateBuffer(
                buffer.dataClass, buffer.data, buffer.userSuppliedCapacity,
                oldCapacity, newCapacity, deferred
            )
        }

        internal fun reallocateBuffer(
            buffer: ParticleBufferInt, oldCapacity: Int, newCapacity: Int,
            deferred: Boolean
        ): IntArray? {
            assert(newCapacity > oldCapacity)
            return BufferUtils.reallocateBuffer(
                buffer.data, buffer.userSuppliedCapacity, oldCapacity,
                newCapacity, deferred
            )
        }
    }
}
