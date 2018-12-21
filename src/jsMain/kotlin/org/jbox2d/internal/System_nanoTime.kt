package org.jbox2d.internal

import kotlin.browser.*

actual fun System_nanoTime(): Long = window.performance.now().toLong()
