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
import org.jbox2d.common.BufferUtils
import org.jbox2d.common.Color3f
import org.jbox2d.common.MathUtils
import org.jbox2d.common.Settings
import org.jbox2d.common.Vec2
import org.jbox2d.internal.*

class DynamicTreeFlatNodes : BroadPhaseStrategy {

    var root: Int = NULL_NODE
    lateinit var aabbs: Array<AABB>
    lateinit var userDatas: Array<Any?>
    lateinit protected var parents: IntArray
    lateinit protected var children1: IntArray
    lateinit protected var children2: IntArray
    lateinit protected var heights: IntArray

    private var nodeCount: Int = 0
    private var nodeCapacity: Int = 16

    private var freeList: Int = 0

    private val drawVecs = Array(4) { Vec2() }

    private var nodeStack = IntArray(20)
    private var nodeStackIndex: Int = 0

    private val r = Vec2()
    private val aabb = AABB()
    private val subInput = RayCastInput()

    override val height: Int
        get() = if (root == NULL_NODE) 0 else heights[root]

    override val maxBalance: Int
        get() {
            var maxBalance = 0
            for (i in 0 until nodeCapacity) {
                if (heights[i] <= 1) continue

                assert(children1[i] != NULL_NODE)

                val child1 = children1[i]
                val child2 = children2[i]
                val balance = MathUtils.abs(heights[child2] - heights[child1])
                maxBalance = MathUtils.max(maxBalance, balance)
            }
            return maxBalance
        }

    override val areaRatio: Float
        get() {
            if (root == NULL_NODE) return 0.0f

            val root = root
            val rootArea = aabbs[root].perimeter

            var totalArea = 0.0f
            for (i in 0 until nodeCapacity) {
                // Free node in pool
                if (heights[i] < 0) continue

                totalArea += aabbs[i].perimeter
            }

            return totalArea / rootArea
        }

    private val combinedAABB = AABB()

    private val color = Color3f()
    private val textVec = Vec2()

    init {
        expandBuffers(0, nodeCapacity)
    }

    private fun expandBuffers(oldSize: Int, newSize: Int) {
        aabbs = BufferUtils.reallocateBuffer({ AABB() }, aabbs, oldSize, newSize)
        userDatas = BufferUtils.reallocateBuffer({ Any() }, userDatas as Array<Any>, oldSize, newSize) as Array<Any?>
        parents = BufferUtils.reallocateBuffer(parents, oldSize, newSize)
        children1 = BufferUtils.reallocateBuffer(children1, oldSize, newSize)
        children2 = BufferUtils.reallocateBuffer(children2, oldSize, newSize)
        heights = BufferUtils.reallocateBuffer(heights, oldSize, newSize)

        // Build a linked list for the free list.
        for (i in oldSize until newSize) {
            aabbs[i] = AABB()
            parents[i] = if (i == newSize - 1) NULL_NODE else i + 1
            heights[i] = -1
            children1[i] = -1
            children2[i] = -1
        }
        freeList = oldSize
    }

    override fun createProxy(aabb: AABB, userData: Any): Int {
        val node = allocateNode()
        // Fatten the aabb
        val nodeAABB = aabbs[node]
        nodeAABB.lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension
        nodeAABB.lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension
        nodeAABB.upperBound.x = aabb.upperBound.x + Settings.aabbExtension
        nodeAABB.upperBound.y = aabb.upperBound.y + Settings.aabbExtension
        userDatas[node] = userData

        insertLeaf(node)

        return node
    }

    override fun destroyProxy(proxyId: Int) {
        assert(proxyId in 0 until nodeCapacity)
        assert(children1[proxyId] == NULL_NODE)

        removeLeaf(proxyId)
        freeNode(proxyId)
    }

    override fun moveProxy(proxyId: Int, aabb: AABB, displacement: Vec2): Boolean {
        assert(proxyId in 0 until nodeCapacity)
        val node = proxyId
        assert(children1[node] == NULL_NODE)

        val nodeAABB = aabbs[node]
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
        assert(proxyId in 0 until nodeCount)
        return userDatas[proxyId]
    }

    override fun getFatAABB(proxyId: Int): AABB {
        assert(proxyId in 0 until nodeCount)
        return aabbs[proxyId]
    }

    override fun query(callback: TreeCallback, aabb: AABB) {
        nodeStackIndex = 0
        nodeStack[nodeStackIndex++] = root

        while (nodeStackIndex > 0) {
            val node = nodeStack[--nodeStackIndex]
            if (node == NULL_NODE) continue

            if (AABB.testOverlap(aabbs[node], aabb)) {
                val child1 = children1[node]
                if (child1 == NULL_NODE) {
                    val proceed = callback.treeCallback(node)
                    if (!proceed) return
                } else {
                    if (nodeStack.size - nodeStackIndex - 2 <= 0) {
                        nodeStack = BufferUtils.reallocateBuffer(nodeStack, nodeStack.size, nodeStack.size * 2)
                    }
                    nodeStack[nodeStackIndex++] = child1
                    nodeStack[nodeStackIndex++] = children2[node]
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
            nodeStack[--nodeStackIndex] = root
            val node = nodeStack[--nodeStackIndex]
            if (node == NULL_NODE) continue

            val nodeAABB = aabbs[node]
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

            val child1 = children1[node]
            if (child1 == NULL_NODE) {
                subInput.p1.x = p1x
                subInput.p1.y = p1y
                subInput.p2.x = p2x
                subInput.p2.y = p2y
                subInput.maxFraction = maxFraction

                val value = callback.raycastCallback(subInput, node)

                if (value == 0.0f) {
                    // The client has terminated the ray cast.
                    return
                }

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
                nodeStack[nodeStackIndex++] = child1
                nodeStack[nodeStackIndex++] = children2[node]
            }
        }
    }

    override fun computeHeight() = computeHeight(root)

    private fun computeHeight(node: Int): Int {
        assert(node in 0 until nodeCapacity)

        if (children1[node] == NULL_NODE) {
            return 0
        }
        val height1 = computeHeight(children1[node])
        val height2 = computeHeight(children2[node])
        return 1 + MathUtils.max(height1, height2)
    }

    /**
     * Validate this tree. For testing.
     */
    fun validate() {
        validateStructure(root)
        validateMetrics(root)

        var freeCount = 0
        var freeNode = freeList
        while (freeNode != NULL_NODE) {
            assert(freeNode in 0 until nodeCapacity)
            freeNode = parents[freeNode]
            ++freeCount
        }

        assert(height == computeHeight())
        assert(nodeCount + freeCount == nodeCapacity)
    }

    // /**
    // * Build an optimal tree. Very expensive. For testing.
    // */
    // public void rebuildBottomUp() {
    // int[] nodes = new int[m_nodeCount];
    // int count = 0;
    //
    // // Build array of leaves. Free the rest.
    // for (int i = 0; i < m_nodeCapacity; ++i) {
    // if (m_nodes[i].height < 0) {
    // // free node in pool
    // continue;
    // }
    //
    // DynamicTreeNode node = m_nodes[i];
    // if (node.isLeaf()) {
    // node.parent = null;
    // nodes[count] = i;
    // ++count;
    // } else {
    // freeNode(node);
    // }
    // }
    //
    // AABB b = new AABB();
    // while (count > 1) {
    // float minCost = Float.MAX_VALUE;
    // int iMin = -1, jMin = -1;
    // for (int i = 0; i < count; ++i) {
    // AABB aabbi = m_nodes[nodes[i]].aabb;
    //
    // for (int j = i + 1; j < count; ++j) {
    // AABB aabbj = m_nodes[nodes[j]].aabb;
    // b.combine(aabbi, aabbj);
    // float cost = b.getPerimeter();
    // if (cost < minCost) {
    // iMin = i;
    // jMin = j;
    // minCost = cost;
    // }
    // }
    // }
    //
    // int index1 = nodes[iMin];
    // int index2 = nodes[jMin];
    // DynamicTreeNode child1 = m_nodes[index1];
    // DynamicTreeNode child2 = m_nodes[index2];
    //
    // DynamicTreeNode parent = allocateNode();
    // parent.child1 = child1;
    // parent.child2 = child2;
    // parent.height = 1 + MathUtils.max(child1.height, child2.height);
    // parent.aabb.combine(child1.aabb, child2.aabb);
    // parent.parent = null;
    //
    // child1.parent = parent;
    // child2.parent = parent;
    //
    // nodes[jMin] = nodes[count - 1];
    // nodes[iMin] = parent.id;
    // --count;
    // }
    //
    // m_root = m_nodes[nodes[0]];
    //
    // validate();
    // }

    private fun allocateNode(): Int {
        if (freeList == NULL_NODE) {
            assert(nodeCount == nodeCapacity)
            nodeCapacity *= 2
            expandBuffers(nodeCount, nodeCapacity)
        }
        assert(freeList != NULL_NODE)
        val node = freeList
        freeList = parents[node]
        parents[node] = NULL_NODE
        children1[node] = NULL_NODE
        heights[node] = 0
        ++nodeCount
        return node
    }

    /**
     * returns a node to the pool
     */
    private fun freeNode(node: Int) {
        assert(node != NULL_NODE)
        assert(0 < nodeCount)
        parents[node] = if (freeList != NULL_NODE) freeList else NULL_NODE
        heights[node] = -1
        freeList = node
        nodeCount--
    }

    private fun insertLeaf(leaf: Int) {
        if (root == NULL_NODE) {
            root = leaf
            parents[root] = NULL_NODE
            return
        }

        // find the best sibling
        val leafAABB = aabbs[leaf]
        var index = root
        while (children1[index] != NULL_NODE) {
            val node = index
            val child1 = children1[node]
            val child2 = children2[node]
            val nodeAABB = aabbs[node]
            val area = nodeAABB.perimeter

            combinedAABB.combine(nodeAABB, leafAABB)
            val combinedArea = combinedAABB.perimeter

            // Cost of creating a new parent for this node and the new leaf
            val cost = 2.0f * combinedArea

            // Minimum cost of pushing the leaf further down the tree
            val inheritanceCost = 2.0f * (combinedArea - area)

            // Cost of descending into child1
            val cost1: Float
            val child1AABB = aabbs[child1]
            if (children1[child1] == NULL_NODE) {
                combinedAABB.combine(leafAABB, child1AABB)
                cost1 = combinedAABB.perimeter + inheritanceCost
            } else {
                combinedAABB.combine(leafAABB, child1AABB)
                val oldArea = child1AABB.perimeter
                val newArea = combinedAABB.perimeter
                cost1 = newArea - oldArea + inheritanceCost
            }

            // Cost of descending into child2
            val cost2: Float
            val child2AABB = aabbs[child2]
            if (children1[child2] == NULL_NODE) {
                combinedAABB.combine(leafAABB, child2AABB)
                cost2 = combinedAABB.perimeter + inheritanceCost
            } else {
                combinedAABB.combine(leafAABB, child2AABB)
                val oldArea = child2AABB.perimeter
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
        val oldParent = parents[sibling]
        val newParent = allocateNode()
        parents[newParent] = oldParent
        userDatas[newParent] = null
        aabbs[newParent].combine(leafAABB, aabbs[sibling])
        heights[newParent] = heights[sibling] + 1

        if (oldParent != NULL_NODE) {
            // The sibling was not the root.
            if (children1[oldParent] == sibling) {
                children1[oldParent] = newParent
            } else {
                children2[oldParent] = newParent
            }

            children1[newParent] = sibling
            children2[newParent] = leaf
            parents[sibling] = newParent
            parents[leaf] = newParent
        } else {
            // The sibling was the root.
            children1[newParent] = sibling
            children2[newParent] = leaf
            parents[sibling] = newParent
            parents[leaf] = newParent
            root = newParent
        }

        // Walk back up the tree fixing heights and AABBs
        index = parents[leaf]
        while (index != NULL_NODE) {
            index = balance(index)

            val child1 = children1[index]
            val child2 = children2[index]

            assert(child1 != NULL_NODE)
            assert(child2 != NULL_NODE)

            heights[index] = 1 + MathUtils.max(heights[child1], heights[child2])
            aabbs[index].combine(aabbs[child1], aabbs[child2])

            index = parents[index]
        }
        // validate();
    }

    private fun removeLeaf(leaf: Int) {
        if (leaf == root) {
            root = NULL_NODE
            return
        }

        val parent = parents[leaf]
        val grandParent = this.parents[parent]
        val parentChild1 = children1[parent]
        val parentChild2 = children2[parent]
        val sibling = if (parentChild1 == leaf) parentChild2 else parentChild1

        if (grandParent != NULL_NODE) {
            // Destroy parent and connect sibling to grandParent.
            if (children1[grandParent] == parent) {
                children1[grandParent] = sibling
            } else {
                children2[grandParent] = sibling
            }
            parents[sibling] = grandParent
            freeNode(parent)

            // Adjust ancestor bounds.
            var index = grandParent
            while (index != NULL_NODE) {
                index = balance(index)

                val child1 = children1[index]
                val child2 = children2[index]

                aabbs[index].combine(aabbs[child1], aabbs[child2])
                heights[index] = 1 + MathUtils.max(heights[child1], heights[child2])

                index = this.parents[index]
            }
        } else {
            root = sibling
            parents[sibling] = NULL_NODE
            freeNode(parent)
        }

        // validate();
    }

    // Perform a left or right rotation if node A is imbalanced.
    // Returns the new root index.
    private fun balance(A: Int): Int {
        assert(A != NULL_NODE)

        if (children1[A] == NULL_NODE || heights[A] < 2) {
            return A
        }

        val B = children1[A]
        val C = children2[A]
        assert(B in 0 until nodeCapacity)
        assert(C in 0 until nodeCapacity)

        val balance = heights[C] - heights[B]

        // Rotate C up
        if (balance > 1) {
            val F = children1[C]
            val G = children2[C]
            // assert (F != null);
            // assert (G != null);
            assert(F in 0 until nodeCapacity)
            assert(G in 0 until nodeCapacity)

            // Swap A and C
            children1[C] = A
            parents[C] = parents[A]
            val cParent = parents[C]
            parents[A] = C

            // A's old parent should point to C
            if (cParent != NULL_NODE) {
                if (children1[cParent] == A) {
                    children1[cParent] = C
                } else {
                    assert(children2[cParent] == A)
                    children2[cParent] = C
                }
            } else {
                root = C
            }

            // Rotate
            if (heights[F] > heights[G]) {
                children2[C] = F
                children2[A] = G
                parents[G] = A
                aabbs[A].combine(aabbs[B], aabbs[G])
                aabbs[C].combine(aabbs[A], aabbs[F])

                heights[A] = 1 + MathUtils.max(heights[B], heights[G])
                heights[C] = 1 + MathUtils.max(heights[A], heights[F])
            } else {
                children2[C] = G
                children2[A] = F
                parents[F] = A
                aabbs[A].combine(aabbs[B], aabbs[F])
                aabbs[C].combine(aabbs[A], aabbs[G])

                heights[A] = 1 + MathUtils.max(heights[B], heights[F])
                heights[C] = 1 + MathUtils.max(heights[A], heights[G])
            }

            return C
        }

        // Rotate B up
        if (balance < -1) {
            val D = children1[B]
            val E = children2[B]
            assert(D in 0 until nodeCapacity)
            assert(E in 0 until nodeCapacity)

            // Swap A and B
            children1[B] = A
            parents[B] = parents[A]
            val Bparent = parents[B]
            parents[A] = B

            // A's old parent should point to B
            if (Bparent != NULL_NODE) {
                if (children1[Bparent] == A) {
                    children1[Bparent] = B
                } else {
                    assert(children2[Bparent] == A)
                    children2[Bparent] = B
                }
            } else {
                root = B
            }

            // Rotate
            if (heights[D] > heights[E]) {
                children2[B] = D
                children1[A] = E
                parents[E] = A
                aabbs[A].combine(aabbs[C], aabbs[E])
                aabbs[B].combine(aabbs[A], aabbs[D])

                heights[A] = 1 + MathUtils.max(heights[C], heights[E])
                heights[B] = 1 + MathUtils.max(heights[A], heights[D])
            } else {
                children2[B] = E
                children1[A] = D
                parents[D] = A
                aabbs[A].combine(aabbs[C], aabbs[D])
                aabbs[B].combine(aabbs[A], aabbs[E])

                heights[A] = 1 + MathUtils.max(heights[C], heights[D])
                heights[B] = 1 + MathUtils.max(heights[A], heights[E])
            }

            return B
        }

        return A
    }

    private fun validateStructure(node: Int) {
        if (node == NULL_NODE) return

        if (node == root) {
            assert(parents[node] == NULL_NODE)
        }

        val child1 = children1[node]
        val child2 = children2[node]

        if (child1 == NULL_NODE) {
            assert(child1 == NULL_NODE)
            assert(child2 == NULL_NODE)
            assert(heights[node] == 0)
            return
        }

        assert(child1 != NULL_NODE && child1 in 0 until nodeCapacity)
        assert(child2 != NULL_NODE && child2 in 0 until nodeCapacity)

        assert(parents[child1] == node)
        assert(parents[child2] == node)

        validateStructure(child1)
        validateStructure(child2)
    }

    private fun validateMetrics(node: Int) {
        if (node == NULL_NODE) return

        val child1 = children1[node]
        val child2 = children2[node]

        if (child1 == NULL_NODE) {
            assert(child1 == NULL_NODE)
            assert(child2 == NULL_NODE)
            assert(heights[node] == 0)
            return
        }

        assert(child1 != NULL_NODE && child1 in 0 until nodeCapacity)
        assert(child2 != child1 && child2 in 0 until nodeCapacity)

        val height1 = heights[child1]
        val height2 = heights[child2]
        val height = 1 + MathUtils.max(height1, height2)
        assert(heights[node] == height)

        val aabb = AABB()
        aabb.combine(aabbs[child1], aabbs[child2])

        assert(aabb.lowerBound == aabbs[node].lowerBound)
        assert(aabb.upperBound == aabbs[node].upperBound)

        validateMetrics(child1)
        validateMetrics(child2)
    }

    override fun drawTree(argDraw: DebugDraw) {
        if (root == NULL_NODE) return
        val height = computeHeight()
        drawTree(argDraw, root, 0, height)
    }

    fun drawTree(argDraw: DebugDraw, node: Int, spot: Int, height: Int) {
        val a = aabbs[node]
        a.getVertices(drawVecs)

        color.set(1f, (height - spot) * 1f / height, (height - spot) * 1f / height)
        argDraw.drawPolygon(drawVecs, 4, color)

        argDraw.viewportTranform!!.getWorldToScreen(a.upperBound, textVec)
        argDraw.drawString(textVec.x, textVec.y, node.toString() + "-" + (spot + 1) + "/" + height, color)

        val c1 = children1[node]
        val c2 = children2[node]
        if (c1 != NULL_NODE) {
            drawTree(argDraw, c1, spot + 1, height)
        }
        if (c2 != NULL_NODE) {
            drawTree(argDraw, c2, spot + 1, height)
        }
    }

    companion object {
        val MAX_STACK_SIZE = 64
        val NULL_NODE = -1
        val INITIAL_BUFFER_LENGTH = 16
    }
}
