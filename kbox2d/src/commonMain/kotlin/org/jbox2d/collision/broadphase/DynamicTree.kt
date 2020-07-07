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
package org.jbox2d.collision.broadphase

import org.jbox2d.callbacks.DebugDraw
import org.jbox2d.callbacks.TreeCallback
import org.jbox2d.callbacks.TreeRayCastCallback
import org.jbox2d.collision.AABB
import org.jbox2d.collision.RayCastInput
import org.jbox2d.common.Color3f
import org.jbox2d.common.MathUtils
import org.jbox2d.common.Settings
import org.jbox2d.common.Vec2
import org.jbox2d.internal.*

/**
 * A dynamic tree arranges data in a binary tree to accelerate queries such as volume queries and
 * ray casts. Leaves are proxies with an AABB. In the tree we expand the proxy AABB by _fatAABBFactor
 * so that the proxy AABB is bigger than the client object. This allows the client object to move by
 * small amounts without triggering a tree update.
 *
 * @author daniel
 */
class DynamicTree : BroadPhaseStrategy {

    private var root: DynamicTreeNode? = null
    private var nodeCount: Int = 0
    private var nodeCapacity: Int = 16
    private var nodes = Array(16) { DynamicTreeNode(it) }.also { nodes ->
        // Build a linked list for the free list.
        for (i in nodeCapacity - 1 downTo 0) {
            nodes[i].parent = if (i == nodeCapacity - 1) null else nodes[i + 1]
            nodes[i].height = -1
        }
    }

    private var freeList: Int = 0

    private val drawVecs = Array(4) { Vec2() }
    private var nodeStack = arrayOfNulls<DynamicTreeNode>(20)
    private var nodeStackIndex = 0

    private val r = Vec2()
    private val aabb = AABB()
    private val subInput = RayCastInput()

    override val height: Int
        get() = if (root == null) 0 else root!!.height

    override val maxBalance: Int
        get() {
            var maxBalance = 0
            for (i in 0 until nodeCapacity) {
                val node = nodes[i]
                if (node.height <= 1) continue

                val child1 = node.child1!!
                val child2 = node.child2!!
                val balance = MathUtils.abs(child2.height - child1.height)
                maxBalance = MathUtils.max(maxBalance, balance)
            }
            return maxBalance
        }

    override val areaRatio: Float
        get() {
            val root = root ?: return 0.0f
            val rootArea = root.aabb.perimeter
            var totalArea = 0.0f

            for (i in 0 until nodeCapacity) {
                val node = nodes[i]
                // Free node in pool
                if (node.height < 0) continue

                totalArea += node.aabb.perimeter
            }

            return totalArea / rootArea
        }

    private val combinedAABB = AABB()

    private val color = Color3f()
    private val textVec = Vec2()

    override fun createProxy(aabb: AABB, userData: Any): Int {
        assert(aabb.isValid)
        val node = allocateNode()
        val proxyId = node.id
        // Fatten the aabb
        val nodeAABB = node.aabb
        nodeAABB.lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension
        nodeAABB.lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension
        nodeAABB.upperBound.x = aabb.upperBound.x + Settings.aabbExtension
        nodeAABB.upperBound.y = aabb.upperBound.y + Settings.aabbExtension
        node.userData = userData

        insertLeaf(proxyId)

        return proxyId
    }

    override fun destroyProxy(proxyId: Int) {
        assert(proxyId in 0 until nodeCapacity)
        val node = nodes[proxyId]
        assert(node.child1 == null)

        removeLeaf(node)
        freeNode(node)
    }

    override fun moveProxy(proxyId: Int, aabb: AABB, displacement: Vec2): Boolean {
        assert(aabb.isValid)
        assert(proxyId in 0 until nodeCapacity)
        val node = nodes[proxyId]
        assert(node.child1 == null)

        val nodeAABB = node.aabb
        if (nodeAABB.contains(aabb)) return false

        removeLeaf(node)

        // Extend AABB
        val lowerBound = nodeAABB.lowerBound
        val upperBound = nodeAABB.upperBound
        lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension
        lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension
        upperBound.x = aabb.upperBound.x + Settings.aabbExtension
        upperBound.y = aabb.upperBound.y + Settings.aabbExtension

        // Predict AABB displacement.
        val dx = displacement.x * Settings.aabbMultiplier
        val dy = displacement.y * Settings.aabbMultiplier
        if (dx < 0.0f) {
            lowerBound.x += dx
        } else {
            upperBound.x += dx
        }

        if (dy < 0.0f) {
            lowerBound.y += dy
        } else {
            upperBound.y += dy
        }

        insertLeaf(proxyId)
        return true
    }

    override fun getUserData(proxyId: Int): Any? {
        assert(proxyId in 0 until nodeCapacity)
        return nodes[proxyId].userData
    }

    override fun getFatAABB(proxyId: Int): AABB {
        assert(proxyId in 0 until nodeCapacity)
        return nodes[proxyId].aabb
    }

    override fun query(callback: TreeCallback, aabb: AABB) {
        assert(aabb.isValid)
        nodeStackIndex = 0
        nodeStack[nodeStackIndex++] = root

        while (nodeStackIndex > 0) {
            val node = nodeStack[--nodeStackIndex] ?: continue

            if (AABB.testOverlap(node.aabb, aabb)) {
                if (node.child1 == null) {
                    val proceed = callback.treeCallback(node.id)
                    if (!proceed) return
                } else {
                    if (nodeStack.size - nodeStackIndex - 2 <= 0) {
                        val newBuffer = arrayOfNulls<DynamicTreeNode>(nodeStack.size * 2)
                        arraycopy(nodeStack, 0, newBuffer, 0, nodeStack.size)
                        nodeStack = newBuffer
                    }
                    nodeStack[nodeStackIndex++] = node.child1
                    nodeStack[nodeStackIndex++] = node.child2
                }
            }
        }
    }

    override fun raycast(callback: TreeRayCastCallback, input: RayCastInput) {
        val p1 = input.p1
        val p2 = input.p2
        val p1x = p1.x
        val p2x = p2.x
        val p1y = p1.y
        val p2y = p2.y
        r.x = p2x - p1x
        r.y = p2y - p1y
        assert(r.x * r.x + r.y * r.y > 0f)
        r.normalize()
        val rx = r.x
        val ry = r.y

        // v is perpendicular to the segment.
        val vx = -1f * ry
        val vy = 1f * rx
        val absVx = MathUtils.abs(vx)
        val absVy = MathUtils.abs(vy)

        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)

        var maxFraction = input.maxFraction

        // Build a bounding box for the segment.
        val segAABB = aabb
        // Vec2 t = p1 + maxFraction * (p2 - p1);
        // before inline
        // temp.set(p2).subLocal(p1).mulLocal(maxFraction).addLocal(p1);
        // Vec2.minToOut(p1, temp, segAABB.lowerBound);
        // Vec2.maxToOut(p1, temp, segAABB.upperBound);
        var tempx = (p2x - p1x) * maxFraction + p1x
        var tempy = (p2y - p1y) * maxFraction + p1y
        segAABB.lowerBound.x = if (p1x < tempx) p1x else tempx
        segAABB.lowerBound.y = if (p1y < tempy) p1y else tempy
        segAABB.upperBound.x = if (p1x > tempx) p1x else tempx
        segAABB.upperBound.y = if (p1y > tempy) p1y else tempy
        // end inline

        nodeStackIndex = 0
        nodeStack[nodeStackIndex++] = root
        while (nodeStackIndex > 0) {
            val node = nodeStack[--nodeStackIndex] ?: continue

            val nodeAABB = node.aabb
            if (!AABB.testOverlap(nodeAABB, segAABB)) {
                continue
            }

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)
            // node.aabb.getCenterToOut(c);
            // node.aabb.getExtentsToOut(h);
            val cx = (nodeAABB.lowerBound.x + nodeAABB.upperBound.x) * .5f
            val cy = (nodeAABB.lowerBound.y + nodeAABB.upperBound.y) * .5f
            val hx = (nodeAABB.upperBound.x - nodeAABB.lowerBound.x) * .5f
            val hy = (nodeAABB.upperBound.y - nodeAABB.lowerBound.y) * .5f
            tempx = p1x - cx
            tempy = p1y - cy
            val separation = MathUtils.abs(vx * tempx + vy * tempy) - (absVx * hx + absVy * hy)
            if (separation > 0.0f) continue

            if (node.child1 == null) {
                subInput.p1.x = p1x
                subInput.p1.y = p1y
                subInput.p2.x = p2x
                subInput.p2.y = p2y
                subInput.maxFraction = maxFraction

                val value = callback.raycastCallback(subInput, node.id)

                // The client has terminated the ray cast.
                if (value == 0.0f) return

                if (value > 0.0f) {
                    // Update segment bounding box.
                    maxFraction = value
                    // temp.set(p2).subLocal(p1).mulLocal(maxFraction).addLocal(p1);
                    // Vec2.minToOut(p1, temp, segAABB.lowerBound);
                    // Vec2.maxToOut(p1, temp, segAABB.upperBound);
                    tempx = (p2x - p1x) * maxFraction + p1x
                    tempy = (p2y - p1y) * maxFraction + p1y
                    segAABB.lowerBound.x = if (p1x < tempx) p1x else tempx
                    segAABB.lowerBound.y = if (p1y < tempy) p1y else tempy
                    segAABB.upperBound.x = if (p1x > tempx) p1x else tempx
                    segAABB.upperBound.y = if (p1y > tempy) p1y else tempy
                }
            } else {
                if (nodeStack.size - nodeStackIndex - 2 <= 0) {
                    val newBuffer = arrayOfNulls<DynamicTreeNode>(nodeStack.size * 2)
                    arraycopy(nodeStack, 0, newBuffer, 0, nodeStack.size)
                    nodeStack = newBuffer
                }
                nodeStack[nodeStackIndex++] = node.child1
                nodeStack[nodeStackIndex++] = node.child2
            }
        }
    }

    override fun computeHeight(): Int = computeHeight(root!!)

    private fun computeHeight(node: DynamicTreeNode): Int {
        assert(node.id in 0 until nodeCapacity)

        if (node.child1 == null) return 0
        val height1 = computeHeight(node.child1!!)
        val height2 = computeHeight(node.child2!!)
        return 1 + MathUtils.max(height1, height2)
    }

    /**
     * Validate this tree. For testing.
     */
    fun validate() {
        validateStructure(root)
        validateMetrics(root)

        var freeCount = 0
        var freeNode: DynamicTreeNode? = if (freeList != NULL_NODE) nodes[freeList] else null
        while (freeNode != null) {
            assert(freeNode.id in 0 until nodeCapacity)
            assert(freeNode === nodes[freeNode.id])
            freeNode = freeNode.parent
            ++freeCount
        }

        assert(height == computeHeight())

        assert(nodeCount + freeCount == nodeCapacity)
    }

    /**
     * Build an optimal tree. Very expensive. For testing.
     */
    fun rebuildBottomUp() {
        val nodes = IntArray(nodeCount)
        var count = 0

        // Build array of leaves. Free the rest.
        for (i in 0 until nodeCapacity) {
            if (this.nodes[i].height < 0) {
                // free node in pool
                continue
            }

            val node = this.nodes[i]
            if (node.child1 == null) {
                node.parent = null
                nodes[count] = i
                ++count
            } else {
                freeNode(node)
            }
        }

        val b = AABB()
        while (count > 1) {
            var minCost = Float.MAX_VALUE
            var iMin = -1
            var jMin = -1
            for (i in 0 until count) {
                val aabbi = this.nodes[nodes[i]].aabb

                for (j in i + 1 until count) {
                    val aabbj = this.nodes[nodes[j]].aabb
                    b.combine(aabbi, aabbj)
                    val cost = b.perimeter
                    if (cost < minCost) {
                        iMin = i
                        jMin = j
                        minCost = cost
                    }
                }
            }

            val index1 = nodes[iMin]
            val index2 = nodes[jMin]
            val child1 = this.nodes[index1]
            val child2 = this.nodes[index2]

            val parent = allocateNode()
            parent.child1 = child1
            parent.child2 = child2
            parent.height = 1 + MathUtils.max(child1.height, child2.height)
            parent.aabb.combine(child1.aabb, child2.aabb)
            parent.parent = null

            child1.parent = parent
            child2.parent = parent

            nodes[jMin] = nodes[count - 1]
            nodes[iMin] = parent.id
            --count
        }

        root = this.nodes[nodes[0]]

        validate()
    }

    private fun allocateNode(): DynamicTreeNode {
        if (freeList == NULL_NODE) {
            assert(nodeCount == nodeCapacity)

            val old = nodes
            nodeCapacity *= 2
            nodes = arrayOfNulls<DynamicTreeNode>(nodeCapacity) as Array<DynamicTreeNode>
            arraycopy(old, 0, nodes, 0, old.size)

            // Build a linked list for the free list.
            for (i in nodeCapacity - 1 downTo nodeCount) {
                nodes[i] = DynamicTreeNode(i)
                nodes[i].parent = if (i == nodeCapacity - 1) null else nodes[i + 1]
                nodes[i].height = -1
            }
            freeList = nodeCount
        }
        val nodeId = freeList
        val treeNode = nodes[nodeId]
        freeList = if (treeNode.parent != null) treeNode.parent!!.id else NULL_NODE

        treeNode.parent = null
        treeNode.child1 = null
        treeNode.child2 = null
        treeNode.height = 0
        treeNode.userData = null
        ++nodeCount
        return treeNode
    }

    /**
     * Returns a node to the pool
     */
    private fun freeNode(node: DynamicTreeNode) {
        assert(nodeCount > 0)
        node.parent = if (freeList != NULL_NODE) nodes[freeList] else null
        node.height = -1
        freeList = node.id
        nodeCount--
    }

    private fun insertLeaf(leafIndex: Int) {
        val leaf = nodes[leafIndex]
        if (root == null) {
            root = leaf
            root!!.parent = null
            return
        }

        // find the best sibling
        val leafAABB = leaf.aabb
        var index = root
        while (index!!.child1 != null) {
            val node = index
            val child1 = node.child1
            val child2 = node.child2

            val area = node.aabb.perimeter

            combinedAABB.combine(node.aabb, leafAABB)
            val combinedArea = combinedAABB.perimeter

            // Cost of creating a new parent for this node and the new leaf
            val cost = 2.0f * combinedArea

            // Minimum cost of pushing the leaf further down the tree
            val inheritanceCost = 2.0f * (combinedArea - area)

            // Cost of descending into child1
            val cost1: Float
            if (child1!!.child1 == null) {
                combinedAABB.combine(leafAABB, child1.aabb)
                cost1 = combinedAABB.perimeter + inheritanceCost
            } else {
                combinedAABB.combine(leafAABB, child1.aabb)
                val oldArea = child1.aabb.perimeter
                val newArea = combinedAABB.perimeter
                cost1 = newArea - oldArea + inheritanceCost
            }

            // Cost of descending into child2
            val cost2: Float
            if (child2!!.child1 == null) {
                combinedAABB.combine(leafAABB, child2.aabb)
                cost2 = combinedAABB.perimeter + inheritanceCost
            } else {
                combinedAABB.combine(leafAABB, child2.aabb)
                val oldArea = child2.aabb.perimeter
                val newArea = combinedAABB.perimeter
                cost2 = newArea - oldArea + inheritanceCost
            }

            // Descend according to the minimum cost.
            if (cost < cost1 && cost < cost2) {
                break
            }

            // Descend
            index = if (cost1 < cost2) child1 else child2
        }

        val sibling = index
        val oldParent = nodes[sibling.id].parent
        val newParent = allocateNode()
        newParent.parent = oldParent
        newParent.userData = null
        newParent.aabb.combine(leafAABB, sibling.aabb)
        newParent.height = sibling.height + 1

        if (oldParent != null) {
            // The sibling was not the root.
            if (oldParent.child1 === sibling) {
                oldParent.child1 = newParent
            } else {
                oldParent.child2 = newParent
            }

            newParent.child1 = sibling
            newParent.child2 = leaf
            sibling.parent = newParent
            leaf.parent = newParent
        } else {
            // The sibling was the root.
            newParent.child1 = sibling
            newParent.child2 = leaf
            sibling.parent = newParent
            leaf.parent = newParent
            root = newParent
        }

        // Walk back up the tree fixing heights and AABBs
        index = leaf.parent
        while (index != null) {
            index = balance(index)

            val child1 = index.child1
            val child2 = index.child2

            assert(child1 != null)
            assert(child2 != null)

            index.height = 1 + MathUtils.max(child1!!.height, child2!!.height)
            index.aabb.combine(child1.aabb, child2.aabb)

            index = index.parent
        }
        // validate();
    }

    private fun removeLeaf(leaf: DynamicTreeNode) {
        if (leaf === root) {
            root = null
            return
        }

        val parent = leaf.parent!!
        val grandParent = parent.parent
        val sibling = if (parent.child1 === leaf) parent.child2!! else parent.child1!!

        if (grandParent != null) {
            // Destroy parent and connect sibling to grandParent.
            if (grandParent.child1 === parent) {
                grandParent.child1 = sibling
            } else {
                grandParent.child2 = sibling
            }
            sibling.parent = grandParent
            freeNode(parent)

            // Adjust ancestor bounds.
            var index = grandParent
            while (index != null) {
                index = balance(index)

                val child1 = index.child1
                val child2 = index.child2

                index.aabb.combine(child1!!.aabb, child2!!.aabb)
                index.height = 1 + MathUtils.max(child1.height, child2.height)

                index = index.parent
            }
        } else {
            root = sibling
            sibling.parent = null
            freeNode(parent)
        }

        // validate();
    }

    // Perform a left or right rotation if node A is imbalanced.
    // Returns the new root index.
    private fun balance(A: DynamicTreeNode): DynamicTreeNode {
        if (A.child1 == null || A.height < 2) return A

        val B = A.child1!!
        val C = A.child2!!
        assert(B.id in 0 until nodeCapacity)
        assert(C.id in 0 until nodeCapacity)

        val balance = C.height - B.height

        // Rotate C up
        if (balance > 1) {
            val F = C.child1!!
            val G = C.child2!!
            assert(F.id in 0 until nodeCapacity)
            assert(G.id in 0 until nodeCapacity)

            // Swap A and C
            C.child1 = A
            C.parent = A.parent
            A.parent = C

            // A's old parent should point to C
            if (C.parent != null) {
                if (C.parent!!.child1 === A) {
                    C.parent!!.child1 = C
                } else {
                    assert(C.parent!!.child2 === A)
                    C.parent!!.child2 = C
                }
            } else {
                root = C
            }

            // Rotate
            if (F.height > G.height) {
                C.child2 = F
                A.child2 = G
                G.parent = A
                A.aabb.combine(B.aabb, G.aabb)
                C.aabb.combine(A.aabb, F.aabb)

                A.height = 1 + MathUtils.max(B.height, G.height)
                C.height = 1 + MathUtils.max(A.height, F.height)
            } else {
                C.child2 = G
                A.child2 = F
                F.parent = A
                A.aabb.combine(B.aabb, F.aabb)
                C.aabb.combine(A.aabb, G.aabb)

                A.height = 1 + MathUtils.max(B.height, F.height)
                C.height = 1 + MathUtils.max(A.height, G.height)
            }

            return C
        }

        // Rotate B up
        if (balance < -1) {
            val D = B.child1!!
            val E = B.child2!!
            assert(0 <= D.id && D.id < nodeCapacity)
            assert(0 <= E.id && E.id < nodeCapacity)

            // Swap A and B
            B.child1 = A
            B.parent = A.parent
            A.parent = B

            // A's old parent should point to B
            if (B.parent != null) {
                if (B.parent!!.child1 === A) {
                    B.parent!!.child1 = B
                } else {
                    assert(B.parent!!.child2 === A)
                    B.parent!!.child2 = B
                }
            } else {
                root = B
            }

            // Rotate
            if (D.height > E.height) {
                B.child2 = D
                A.child1 = E
                E.parent = A
                A.aabb.combine(C.aabb, E.aabb)
                B.aabb.combine(A.aabb, D.aabb)

                A.height = 1 + MathUtils.max(C.height, E.height)
                B.height = 1 + MathUtils.max(A.height, D.height)
            } else {
                B.child2 = E
                A.child1 = D
                D.parent = A
                A.aabb.combine(C.aabb, D.aabb)
                B.aabb.combine(A.aabb, E.aabb)

                A.height = 1 + MathUtils.max(C.height, D.height)
                B.height = 1 + MathUtils.max(A.height, E.height)
            }

            return B
        }

        return A
    }

    private fun validateStructure(node: DynamicTreeNode?) {
        if (node == null) return
        assert(node === nodes[node.id])

        if (node === root) {
            assert(node.parent == null)
        }

        val child1 = node.child1
        val child2 = node.child2

        if (node.child1 == null) {
            assert(child1 == null)
            assert(child2 == null)
            assert(node.height == 0)
            return
        }

        assert(child1 != null && child1.id in 0 until nodeCapacity)
        assert(child2 != null && child2.id in 0 until nodeCapacity)

        assert(child1!!.parent === node)
        assert(child2!!.parent === node)

        validateStructure(child1)
        validateStructure(child2)
    }

    private fun validateMetrics(node: DynamicTreeNode?) {
        if (node == null) return

        val child1 = node.child1
        val child2 = node.child2

        if (node.child1 == null) {
            assert(child1 == null)
            assert(child2 == null)
            assert(node.height == 0)
            return
        }

        assert(child1 != null && 0 <= child1.id && child1.id < nodeCapacity)
        assert(child2 != null && 0 <= child2.id && child2.id < nodeCapacity)

        val height1 = child1!!.height
        val height2 = child2!!.height
        val height = 1 + MathUtils.max(height1, height2)
        assert(node.height == height)

        val aabb = AABB()
        aabb.combine(child1.aabb, child2.aabb)

        assert(aabb.lowerBound == node.aabb.lowerBound)
        assert(aabb.upperBound == node.aabb.upperBound)

        validateMetrics(child1)
        validateMetrics(child2)
    }

    override fun drawTree(argDraw: DebugDraw) {
        if (root == null) return
        val height = computeHeight()
        drawTree(argDraw, root!!, 0, height)
    }

    fun drawTree(argDraw: DebugDraw, node: DynamicTreeNode, spot: Int, height: Int) {
        node.aabb.getVertices(drawVecs)

        color.set(1f, (height - spot) * 1f / height, (height - spot) * 1f / height)
        argDraw.drawPolygon(drawVecs, 4, color)

        argDraw.viewportTranform!!.getWorldToScreen(node.aabb.upperBound, textVec)
        argDraw.drawString(textVec.x, textVec.y, node.id.toString() + "-" + (spot + 1) + "/" + height, color)

        if (node.child1 != null) {
            drawTree(argDraw, node.child1!!, spot + 1, height)
        }
        if (node.child2 != null) {
            drawTree(argDraw, node.child2!!, spot + 1, height)
        }
    }

    companion object {
        val MAX_STACK_SIZE = 64
        val NULL_NODE = -1
    }
}
