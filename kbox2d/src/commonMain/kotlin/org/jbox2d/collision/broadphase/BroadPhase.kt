package org.jbox2d.collision.broadphase

import org.jbox2d.callbacks.DebugDraw
import org.jbox2d.callbacks.PairCallback
import org.jbox2d.callbacks.TreeCallback
import org.jbox2d.callbacks.TreeRayCastCallback
import org.jbox2d.collision.AABB
import org.jbox2d.collision.RayCastInput
import org.jbox2d.common.Vec2

interface BroadPhase {

    /** Number of proxies. */
    val proxyCount: Int

    /** Height of the embedded tree. */
    val treeHeight: Int

    val treeBalance: Int

    val treeQuality: Float

    /**
     * Create a proxy with an initial [aabb]. Pairs are not reported until [updatePairs] is called.
     */
    fun createProxy(aabb: AABB, userData: Any): Int

    /**
     * Destroy a proxy. It is up to the client to remove any pairs.
     */
    fun destroyProxy(proxyId: Int)

    /**
     * Call [moveProxy] as many times as you like, then when you are done call [updatePairs] to finalize
     * the proxy pairs (for your time step).
     */
    fun moveProxy(proxyId: Int, aabb: AABB, displacement: Vec2)

    fun touchProxy(proxyId: Int)

    fun getUserData(proxyId: Int): Any?

    fun getFatAABB(proxyId: Int): AABB

    fun testOverlap(proxyIdA: Int, proxyIdB: Int): Boolean

    fun drawTree(argDraw: DebugDraw)

    /**
     * Update the pairs. This results in pair callbacks. This can only add pairs.
     */
    fun updatePairs(callback: PairCallback)

    /**
     * Query an [aabb] for overlapping proxies. The [callback] is called for each proxy that
     * overlaps the supplied [aabb].
     */
    fun query(callback: TreeCallback, aabb: AABB)

    /**
     * Ray-cast against the proxies in the tree. This relies on the callback to perform an exact
     * ray-cast in the case where the proxy contains a shape. The callback also performs the any
     * collision filtering. This has performance roughly equal to k * log(n), where k is the number of
     * collisions and n is the number of proxies in the tree.
     *
     * @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
     * @param callback a callback class that is called for each proxy that is hit by the ray.
     */
    fun raycast(callback: TreeRayCastCallback, input: RayCastInput)

    companion object {
        val NULL_PROXY = -1
    }
}
