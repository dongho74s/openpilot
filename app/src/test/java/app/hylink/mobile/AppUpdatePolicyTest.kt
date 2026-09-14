package app.hylink.mobile

import org.json.JSONObject
import org.junit.Assert.*
import org.junit.Test
import java.io.ByteArrayInputStream
import java.io.ByteArrayOutputStream
import java.io.IOException
import java.io.InputStream
import java.security.MessageDigest

class AppUpdatePolicyTest {
    private val payload = "synthetic APK bytes for policy tests".toByteArray()
    private val packageName = "app.hylink.mobile.debug"
    private fun manifest() = JSONObject().put("schemaVersion", 1).put("channel", "hylink-dev")
        .put("packageName", packageName).put("versionCode", 13).put("versionName", "1.6.5-app-updates-debug")
        .put("minSdk", 24).put("size", payload.size)
        .put("apkUrl", "https://raw.githubusercontent.com/leehyuk1108/carrotpilot/" + "a".repeat(40) + "/downloads/hylink-dev-1.6.5.apk")
        .put("sha256", AppUpdatePolicy.hex(MessageDigest.getInstance("SHA-256").digest(payload)))
        .put("notes", "앱 업데이트 추가")
    private fun release() = AppUpdatePolicy.parse(manifest().toString(), packageName)
    private fun rejected(action: () -> Unit) {
        try { action(); fail("Expected rejection") } catch (_: IOException) { /* fail closed */ }
    }

    @Test fun validManifestAndVersionRules() {
        val release = release()
        assertTrue(AppUpdatePolicy.newer(release, 12, 36))
        assertFalse(AppUpdatePolicy.newer(release, 13, 36))
        assertFalse(AppUpdatePolicy.newer(release, 14, 36))
        rejected { AppUpdatePolicy.newer(release, 12, 23) }
    }

    @Test fun rejectsOtherSourcesAndMutableReferences() {
        val approved = manifest().getString("apkUrl")
        listOf(approved.replace("carrotpilot", "Wayon"), approved.replace("https:", "http:"),
            approved.replace("raw.githubusercontent.com", "raw.githubusercontent.com.evil.example"),
            approved.replace("a".repeat(40), "refs/heads/hylink-app"), approved + "?token=secret",
            approved + "#fragment", approved.replace("/downloads/", "/downloads/../"),
            approved.replace("githubusercontent.com/", "githubusercontent.com:443/"),
            approved.replace("https://", "https://name@"), approved.replace(".apk", ".zip"))
            .forEach { value -> rejected { AppUpdatePolicy.parse(manifest().put("apkUrl", value).toString(), packageName) } }
    }

    @Test fun rejectsWrongPackageChannelAndSchema() {
        for ((key, value) in listOf("packageName" to "com.example.other", "channel" to "production", "schemaVersion" to 2)) {
            rejected { AppUpdatePolicy.parse(manifest().put(key, value).toString(), packageName) }
        }
        rejected { AppUpdatePolicy.parse(manifest().toString(), "app.hylink.mobile") }
    }

    @Test fun rejectsCoercedFractionalAndOutOfRangeNumbers() {
        for (key in listOf("versionCode", "size", "minSdk", "schemaVersion")) {
            for (value in listOf<Any>("13", 13.5, true, -1, Long.MAX_VALUE)) {
                rejected { AppUpdatePolicy.parse(manifest().put(key, value).toString(), packageName) }
            }
        }
        rejected { AppUpdatePolicy.parse(manifest().put("size", AppUpdatePolicy.MAX_APK_BYTES + 1).toString(), packageName) }
    }

    @Test fun rejectsMissingMalformedAndOversizedManifest() {
        for (key in listOf("packageName", "versionCode", "apkUrl", "sha256", "size")) {
            val json = manifest().apply { remove(key) }
            rejected { AppUpdatePolicy.parse(json.toString(), packageName) }
        }
        rejected { AppUpdatePolicy.parse("not json", packageName) }
        rejected { AppUpdatePolicy.parse(manifest().put("notes", "x".repeat(1001)).toString(), packageName) }
        rejected { AppUpdatePolicy.parse(" ".repeat(32769), packageName) }
        rejected { AppUpdatePolicy.parse(manifest().put("sha256", "zz".repeat(32)).toString(), packageName) }
        rejected { AppUpdatePolicy.parse(manifest().put("versionName", "bad\nname").toString(), packageName) }
    }

    @Test fun verifiesExactBytesAndProgress() {
        val output = ByteArrayOutputStream()
        val progress = mutableListOf<Int>()
        AppUpdatePolicy.copyVerified(ByteArrayInputStream(payload), output, release(), progress = { progress.add(it) })
        assertArrayEquals(payload, output.toByteArray())
        assertEquals(100, progress.last())
    }

    @Test fun rejectsTruncatedExtraOrModifiedData() {
        for (bytes in listOf(payload.dropLast(1).toByteArray(), payload + 1.toByte(), payload.copyOf().apply { this[0] = 0 })) {
            rejected { AppUpdatePolicy.copyVerified(ByteArrayInputStream(bytes), ByteArrayOutputStream(), release()) }
        }
    }

    @Test fun rejectsCancellationAndReadFailure() {
        rejected { AppUpdatePolicy.copyVerified(ByteArrayInputStream(payload), ByteArrayOutputStream(), release(), { true }) }
        val broken = object : InputStream() { override fun read(): Int = throw IOException("network interrupted") }
        rejected { AppUpdatePolicy.copyVerified(broken, ByteArrayOutputStream(), release()) }
    }

    @Test fun exactIdentityPasses() {
        val r = release()
        AppUpdatePolicy.verifyIdentity(r, 12, packageName, 13, r.versionName, 24, setOf("certificate"), setOf("certificate"), 36)
    }

    @Test fun rejectsDifferentSignerMissingSignerAndMultipleSignerMismatch() {
        val r = release()
        for ((old, new) in listOf(setOf("good") to setOf("bad"), emptySet<String>() to emptySet(),
            setOf("good") to emptySet(), setOf("good") to setOf("good", "extra"))) {
            rejected { AppUpdatePolicy.verifyIdentity(r, 12, packageName, 13, r.versionName, 24, old, new, 36) }
        }
    }

    @Test fun rejectsOtherPackageArchiveVersionNameSdkAndDowngrades() {
        val r = release()
        fun verify(installed: Long = 12, pkg: String = packageName, code: Long = 13, name: String = r.versionName, sdk: Int = 24) =
            AppUpdatePolicy.verifyIdentity(r, installed, pkg, code, name, sdk, setOf("good"), setOf("good"), 36)
        rejected { verify(installed = 13) }
        rejected { verify(installed = 14) }
        rejected { verify(pkg = "different.app") }
        rejected { verify(code = 14) }
        rejected { verify(name = "mislabelled") }
        rejected { verify(sdk = 25) }
    }
}
