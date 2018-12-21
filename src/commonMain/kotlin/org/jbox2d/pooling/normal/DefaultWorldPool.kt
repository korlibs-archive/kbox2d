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
/**
 * Created at 3:26:14 AM Jan 11, 2011
 */
package org.jbox2d.pooling.normal

import org.jbox2d.collision.AABB
import org.jbox2d.collision.Collision
import org.jbox2d.collision.Distance
import org.jbox2d.collision.TimeOfImpact
import org.jbox2d.common.Mat22
import org.jbox2d.common.Mat33
import org.jbox2d.common.Rot
import org.jbox2d.common.Settings
import org.jbox2d.common.Vec2
import org.jbox2d.common.Vec3
import org.jbox2d.dynamics.contacts.ChainAndCircleContact
import org.jbox2d.dynamics.contacts.ChainAndPolygonContact
import org.jbox2d.dynamics.contacts.CircleContact
import org.jbox2d.dynamics.contacts.Contact
import org.jbox2d.dynamics.contacts.EdgeAndCircleContact
import org.jbox2d.dynamics.contacts.EdgeAndPolygonContact
import org.jbox2d.dynamics.contacts.PolygonAndCircleContact
import org.jbox2d.dynamics.contacts.PolygonContact
import org.jbox2d.internal.*
import org.jbox2d.pooling.IDynamicStack
import org.jbox2d.pooling.IWorldPool

/**
 * Provides object pooling for all objects used in the engine. Objects retrieved from here should
 * only be used temporarily, and then pushed back (with the exception of arrays).
 *
 * @author Daniel Murphy
 */
class DefaultWorldPool(argSize: Int, argContainerSize: Int) : IWorldPool {

    private val vecs: OrderedStack<Vec2>
    private val vec3s: OrderedStack<Vec3>
    private val mats: OrderedStack<Mat22>
    private val mat33s: OrderedStack<Mat33>
    private val aabbs: OrderedStack<AABB>
    private val rots: OrderedStack<Rot>

    private val afloats = HashMap<Int, FloatArray>()
    private val aints = HashMap<Int, IntArray>()
    private val avecs = HashMap<Int, Array<Vec2>>()

    private val world = this

    private val pcstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return PolygonContact(world)
        }

        override fun newArray(size: Int): Array<Contact> {
            return arrayOfNulls<PolygonContact>(size) as Array<Contact>
        }
    }

    private val ccstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return CircleContact(world)
        }

        override fun newArray(size: Int): Array<Contact> {
            return arrayOfNulls<CircleContact>(size) as Array<Contact>
        }
    }

    private val cpstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return PolygonAndCircleContact(world)
        }

        override fun newArray(size: Int): Array<Contact> {
            return arrayOfNulls<PolygonAndCircleContact>(size) as Array<Contact>
        }
    }

    private val ecstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return EdgeAndCircleContact(world)
        }

        override fun newArray(size: Int): Array<Contact> {
            return arrayOfNulls<EdgeAndCircleContact>(size) as Array<Contact>
        }
    }

    private val epstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return EdgeAndPolygonContact(world)
        }

        override fun newArray(size: Int): Array<Contact> {
            return arrayOfNulls<EdgeAndPolygonContact>(size) as Array<Contact>
        }
    }

    private val chcstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return ChainAndCircleContact(world)
        }

        override fun newArray(size: Int): Array<Contact> {
            return arrayOfNulls<ChainAndCircleContact>(size) as Array<Contact>
        }
    }

    private val chpstack = object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
        override fun newInstance(): Contact {
            return ChainAndPolygonContact(world)
        }

        override fun newArray(size: Int): Array<Contact> {
            return arrayOfNulls<ChainAndPolygonContact>(size) as Array<Contact>
        }
    }

    override val collision: Collision
    override val timeOfImpact: TimeOfImpact
    override val distance: Distance

    override val polyContactStack: IDynamicStack<Contact>
        get() = pcstack

    override val circleContactStack: IDynamicStack<Contact>
        get() = ccstack

    override val polyCircleContactStack: IDynamicStack<Contact>
        get() = cpstack

    override val edgeCircleContactStack: IDynamicStack<Contact>
        get() = ecstack

    override val edgePolyContactStack: IDynamicStack<Contact>
        get() = epstack

    override val chainCircleContactStack: IDynamicStack<Contact>
        get() = chcstack

    override val chainPolyContactStack: IDynamicStack<Contact>
        get() = chpstack

    init {
        vecs = object : OrderedStack<Vec2>(argSize, argContainerSize) {
            override fun newInstance(): Vec2 {
                return Vec2()
            }
        }
        vec3s = object : OrderedStack<Vec3>(argSize, argContainerSize) {
            override fun newInstance(): Vec3 {
                return Vec3()
            }
        }
        mats = object : OrderedStack<Mat22>(argSize, argContainerSize) {
            override fun newInstance(): Mat22 {
                return Mat22()
            }
        }
        aabbs = object : OrderedStack<AABB>(argSize, argContainerSize) {
            override fun newInstance(): AABB {
                return AABB()
            }
        }
        rots = object : OrderedStack<Rot>(argSize, argContainerSize) {
            override fun newInstance(): Rot {
                return Rot()
            }
        }
        mat33s = object : OrderedStack<Mat33>(argSize, argContainerSize) {
            override fun newInstance(): Mat33 {
                return Mat33()
            }
        }

        distance = Distance()
        collision = Collision(this)
        timeOfImpact = TimeOfImpact(this)
    }

    override fun popVec2(): Vec2 {
        return vecs.pop()
    }

    override fun popVec2(argNum: Int): Array<Vec2> {
        return vecs.pop(argNum)
    }

    override fun pushVec2(argNum: Int) {
        vecs.push(argNum)
    }

    override fun popVec3(): Vec3 {
        return vec3s.pop()
    }

    override fun popVec3(argNum: Int): Array<Vec3> {
        return vec3s.pop(argNum)
    }

    override fun pushVec3(argNum: Int) {
        vec3s.push(argNum)
    }

    override fun popMat22(): Mat22 {
        return mats.pop()
    }

    override fun popMat22(argNum: Int): Array<Mat22> {
        return mats.pop(argNum)
    }

    override fun pushMat22(argNum: Int) {
        mats.push(argNum)
    }

    override fun popMat33(): Mat33 {
        return mat33s.pop()
    }

    override fun pushMat33(argNum: Int) {
        mat33s.push(argNum)
    }

    override fun popAABB(): AABB {
        return aabbs.pop()
    }

    override fun popAABB(argNum: Int): Array<AABB> {
        return aabbs.pop(argNum)
    }

    override fun pushAABB(argNum: Int) {
        aabbs.push(argNum)
    }

    override fun popRot(): Rot {
        return rots.pop()
    }

    override fun pushRot(num: Int) {
        rots.push(num)
    }

    override fun getFloatArray(argLength: Int): FloatArray {
        if (!afloats.containsKey(argLength)) {
            afloats[argLength] = FloatArray(argLength)
        }

        assert(afloats[argLength]!!.size == argLength) { "Array not built with correct length" }
        return afloats[argLength]!!
    }

    override fun getIntArray(argLength: Int): IntArray {
        if (!aints.containsKey(argLength)) {
            aints[argLength] = IntArray(argLength)
        }

        assert(aints[argLength]!!.size == argLength) { "Array not built with correct length" }
        return aints[argLength]!!
    }

    override fun getVec2Array(argLength: Int): Array<Vec2> {
        if (!avecs.containsKey(argLength)) {
            avecs[argLength] = Array<Vec2>(argLength) { Vec2() }
        }

        assert(avecs[argLength]!!.size == argLength) { "Array not built with correct length" }
        return avecs[argLength]!!
    }
}
