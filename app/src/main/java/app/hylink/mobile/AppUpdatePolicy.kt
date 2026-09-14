package app.hylink.mobile

import org.json.JSONObject
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.security.MessageDigest

internal class UpdateRejected(message: String) : IOException(message)

internal data class AppRelease(
    val packageName: String,
    val versionCode: Long,
    val versionName: String,
    val minSdk: Int,
    val apkUrl: String,
    val size: Long,
    val sha256: String,
    val notes: String,
)

/** No caller-supplied repository/URL, Cloud key, downgrade or alternate package. */
internal object AppUpdatePolicy {
    const val MANIFEST_URL = "https://raw.githubusercontent.com/leehyuk1108/carrotpilot/refs/heads/hylink-app/downloads/update.json"
    const val MAX_MANIFEST_BYTES = 32 * 1024
    const val MAX_APK_BYTES = 80L * 1024 * 1024
    private val apkLocation = Regex("https://raw\\.githubusercontent\\.com/leehyuk1108/carrotpilot/[0-9a-f]{40}/downloads/hylink-dev-[0-9]+\\.[0-9]+\\.[0-9]+\\.apk")

    fun parse(text: String, expectedPackage: String): AppRelease {
        try {
            if (text.toByteArray(Charsets.UTF_8).size > MAX_MANIFEST_BYTES) reject()
            val json = JSONObject(text)
            fun integer(name: String): Long {
                val value = json.get(name)
                if (value !is Int && value !is Long) reject()
                return (value as Number).toLong()
            }
            if (integer("schemaVersion") != 1L || json.getString("channel") != "hylink-dev") reject()
            val name = json.getString("packageName")
            val code = integer("versionCode")
            val version = json.getString("versionName")
            val sdk = integer("minSdk")
            val url = json.getString("apkUrl")
            val size = integer("size")
            val hash = json.getString("sha256")
            val notes = json.optString("notes", "")
            if (name != expectedPackage || expectedPackage != "app.hylink.mobile.debug" ||
                code !in 1..Int.MAX_VALUE.toLong() || sdk !in 24..100 ||
                !Regex("[0-9A-Za-z.+_-]{1,64}").matches(version) ||
                !apkLocation.matches(url) || size !in 1..MAX_APK_BYTES ||
                !Regex("[0-9a-f]{64}").matches(hash) || notes.length > 1000) reject()
            return AppRelease(name, code, version, sdk.toInt(), url, size, hash, notes)
        } catch (error: UpdateRejected) {
            throw error
        } catch (_: Exception) {
            reject()
        }
    }

    fun newer(release: AppRelease, installedVersion: Long, sdk: Int): Boolean {
        if (release.minSdk > sdk) throw UpdateRejected("이 버전은 Android 업데이트가 먼저 필요해요.")
        return release.versionCode > installedVersion
    }

    fun verifyIdentity(release: AppRelease, installedVersion: Long, archivePackage: String,
                       archiveVersion: Long, archiveName: String, archiveMinSdk: Int,
                       installedSigners: Set<String>, archiveSigners: Set<String>, sdk: Int) {
        if (!newer(release, installedVersion, sdk) || archiveVersion != release.versionCode ||
            archiveName != release.versionName || archivePackage != release.packageName ||
            archiveMinSdk != release.minSdk || installedSigners.isEmpty() ||
            archiveSigners != installedSigners) {
            throw UpdateRejected("앱 버전이나 서명이 일치하지 않아 설치하지 않았어요.")
        }
    }

    /** Exact size + SHA-256; callers delete the partial file on every failure. */
    fun copyVerified(input: InputStream, output: OutputStream, release: AppRelease,
                     cancelled: () -> Boolean = { false }, progress: (Int) -> Unit = {}) {
        val digest = MessageDigest.getInstance("SHA-256")
        val buffer = ByteArray(64 * 1024)
        var count = 0L
        var lastProgress = -1
        while (true) {
            if (cancelled()) throw IOException("cancelled")
            val read = input.read(buffer)
            if (read == -1) break
            count += read
            if (count > release.size || count > MAX_APK_BYTES) throw UpdateRejected("업데이트 파일 크기가 일치하지 않아요.")
            digest.update(buffer, 0, read)
            output.write(buffer, 0, read)
            val percent = (count * 100 / release.size).toInt()
            if (percent != lastProgress) { progress(percent); lastProgress = percent }
        }
        if (count != release.size || hex(digest.digest()) != release.sha256) {
            throw UpdateRejected("파일 검증에 실패했어요. 다시 다운로드해 주세요.")
        }
    }

    fun hex(bytes: ByteArray): String = bytes.joinToString("") { "%02x".format(it.toInt() and 255) }
    private fun reject(): Nothing = throw UpdateRejected("업데이트 정보를 확인할 수 없어요. 잠시 후 다시 시도해 주세요.")
}
