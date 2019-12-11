package org.jbox2d.dynamics

import org.jbox2d.dynamics.joints.*

inline fun World.forEachBody(callback: (Body) -> Unit) {
    var node = bodyList
    while (node != null) {
        callback(node)
        node = node.m_next
    }
}

inline fun World.forEachJoint(callback: (Joint) -> Unit) {
    var node = jointList
    while (node != null) {
        callback(node)
        node = node.m_next
    }
}
