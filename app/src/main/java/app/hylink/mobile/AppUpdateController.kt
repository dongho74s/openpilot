package app.hylink.mobile

import android.content.Context
import android.content.Intent
import android.content.pm.PackageInfo
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.provider.Settings
import androidx.core.content.FileProvider
import okhttp3.Call
import okhttp3.OkHttpClient
import okhttp3.Request
import org.json.JSONObject
import java.io.ByteArrayOutputStream
import java.io.File
import java.io.IOException
import java.io.OutputStream
import java.security.MessageDigest
import java.util.concurrent.Executors
import java.util.concurrent.TimeUnit

/** Foreground-only updater. Installation always uses Android's user confirmation. */
internal class AppUpdateController(
    private val context: Context,
    private val canInteract: () -> Boolean,
    private val onState: (JSONObject) -> Unit,
    private val launchInstaller: (Intent) -> Unit,
    private val launchPermission: (Intent) -> Unit,
) {
    private val handler = Handler(Looper.getMainLooper())
    private val executor = Executors.newSingleThreadExecutor()
    private val client = OkHttpClient.Builder().followRedirects(false).followSslRedirects(false)
        .connectTimeout(10, TimeUnit.SECONDS).readTimeout(20, TimeUnit.SECONDS)
        .callTimeout(120, TimeUnit.SECONDS).build()
    private val preferences = context.getSharedPreferences("HylinkAppUpdates", Context.MODE_PRIVATE)
    @Volatile private var generation = 0
    @Volatile private var closed = false
    @Volatile private var call: Call? = null
    private var busy = false // UI thread owns state, release and verifiedFile.
    private var release: AppRelease? = null
    private var verifiedFile: File? = null
    private var phase = "idle"
    private var snapshot = state("idle", "앱을 열면 새 버전을 자동으로 확인해요.")

    init {
        executor.execute {
            val cutoff = System.currentTimeMillis() - TimeUnit.DAYS.toMillis(1)
            File(context.cacheDir, "app-updates").listFiles()?.filter {
                it.isFile && it.name.matches(Regex("hylink-[0-9]+\\.apk")) && it.lastModified() < cutoff
            }?.forEach { it.delete() }
        }
        // Cached public release metadata contains no account or vehicle information.
        runCatching {
            AppUpdatePolicy.parse(preferences.getString("manifest", "").orEmpty(), context.packageName)
        }.getOrNull()?.let {
            if (it.versionCode > BuildConfig.VERSION_CODE && it.minSdk <= Build.VERSION.SDK_INT) {
                release = it
                snapshot = state("available", "새 버전을 사용할 수 있어요.")
                phase = "available"
            }
        }
    }

    fun emit() = onState(snapshot)

    fun check(manual: Boolean) {
        if (busy || closed || !canInteract() || (!manual && verifiedFile != null)) return
        val now = System.currentTimeMillis()
        val last = preferences.getLong("lastAttempt", 0)
        if (!manual && now - last in 0 until TimeUnit.HOURS.toMillis(6)) { emit(); return }
        preferences.edit().putLong("lastAttempt", now).apply()
        val job = begin("checking", "새 버전을 확인하고 있어요.")
        executor.execute {
            try {
                val latest = fetchRelease(job)
                ui(job) {
                    release = latest
                    busy = false
                    show(if (latest.versionCode > BuildConfig.VERSION_CODE) "available" else "current",
                         if (latest.versionCode > BuildConfig.VERSION_CODE) "새 버전을 사용할 수 있어요." else "최신 버전이에요.")
                }
            } catch (error: Exception) { fail(job, error) }
        }
    }

    fun download() {
        if (busy || closed || !canInteract()) return
        val selected = release ?: return check(true)
        if (selected.versionCode <= BuildConfig.VERSION_CODE) return check(true)
        val job = begin("downloading", "업데이트 파일을 준비하고 있어요.")
        executor.execute {
            var temporary: File? = null
            try {
                val latest = fetchRelease(job) // Recheck publication before downloading.
                if (latest != selected) {
                    ui(job) { release = latest; busy = false; show("idle", "배포 정보가 바뀌었어요. 다시 확인해 주세요.") }
                    return@execute
                }
                val directory = File(context.cacheDir, "app-updates").apply { mkdirs() }
                val target = File.createTempFile("hylink-", ".apk", directory)
                temporary = target
                val request = Request.Builder().url(latest.apkUrl).header("Accept", "application/octet-stream").build()
                execute(job, request) { response ->
                    if (!response.isSuccessful) throw IOException("download HTTP ${response.code}")
                    val body = response.body ?: throw IOException("empty download")
                    val length = body.contentLength()
                    if (length >= 0 && length != latest.size) throw UpdateRejected("업데이트 파일 크기가 일치하지 않아요.")
                    target.outputStream().use { output ->
                        body.byteStream().use { input ->
                            AppUpdatePolicy.copyVerified(input, output, latest, { cancelled(job) }) { percent ->
                                ui(job) { show("downloading", "업데이트를 다운로드하고 있어요.", percent) }
                            }
                        }
                        output.fd.sync()
                    }
                }
                ui(job) { show("verifying", "파일과 앱 서명을 확인하고 있어요.") }
                verifyArchive(target, latest)
                val ready = target
                temporary = null
                handler.post {
                    if (cancelled(job)) ready.delete() else {
                        verifiedFile?.delete()
                        verifiedFile = ready
                        busy = false
                        show("ready", "검증 완료 · 설치 화면에서 한 번 더 확인해 주세요.")
                    }
                }
            } catch (error: Exception) { fail(job, error) }
            finally { temporary?.delete() }
        }
    }

    fun install() {
        if (busy || closed || !canInteract()) return
        val file = verifiedFile ?: return download()
        val selected = release ?: return check(true)
        if (Build.VERSION.SDK_INT >= 26 && !context.packageManager.canRequestPackageInstalls()) {
            show("permission", "Hylink의 앱 설치를 허용한 뒤 돌아와 ‘설치’를 눌러 주세요.")
            try {
                launchPermission(Intent(Settings.ACTION_MANAGE_UNKNOWN_APP_SOURCES, Uri.parse("package:${context.packageName}")))
            } catch (_: Exception) { show("ready", "설정에서 Hylink의 앱 설치 허용을 확인해 주세요.") }
            return
        }
        val job = begin("verifying", "설치 전 파일을 다시 확인하고 있어요.")
        executor.execute {
            try {
                file.inputStream().use { AppUpdatePolicy.copyVerified(it, DISCARD, selected, { cancelled(job) }) }
                verifyArchive(file, selected)
                ui(job) {
                    busy = false
                    if (!canInteract()) { show("ready", "앱으로 돌아와 설치를 눌러 주세요."); return@ui }
                    val uri = FileProvider.getUriForFile(context, "${context.packageName}.updates", file)
                    @Suppress("DEPRECATION")
                    val intent = Intent(Intent.ACTION_INSTALL_PACKAGE).setDataAndType(uri, "application/vnd.android.package-archive")
                        .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
                        .putExtra(Intent.EXTRA_RETURN_RESULT, true)
                    try { show("installing", "Android 설치 화면에서 업데이트를 확인해 주세요."); launchInstaller(intent) }
                    catch (_: Exception) { show("ready", "설치 화면을 열지 못했어요. 다시 시도해 주세요.") }
                }
            } catch (error: Exception) {
                file.delete()
                ui(job) { verifiedFile = null }
                fail(job, error)
            }
        }
    }

    fun installerReturned() {
        if (verifiedFile != null) show("ready", "설치가 완료되지 않았다면 다시 설치를 눌러 주세요.")
    }

    fun permissionReturned() {
        if (verifiedFile != null) show("ready", if (Build.VERSION.SDK_INT < 26 || context.packageManager.canRequestPackageInstalls())
            "설치가 허용됐어요. 설치를 눌러 계속해 주세요." else "앱 설치가 허용되지 않았어요. 설치를 눌러 다시 확인해 주세요.")
    }

    fun pause() { if (busy) cancel() }

    fun cancel() {
        if (!busy) return
        generation++
        call?.cancel()
        busy = false
        show(if (verifiedFile != null) "ready" else if ((release?.versionCode ?: 0) > BuildConfig.VERSION_CODE) "available" else "idle",
             "작업을 중지했어요. 원할 때 다시 시도할 수 있어요.")
    }

    fun close() {
        closed = true
        generation++
        call?.cancel()
        executor.shutdownNow()
        // Only this updater's private temporary APK; never preferences or saved media.
        if (phase != "installing") verifiedFile?.delete()
    }

    private fun begin(next: String, message: String): Int {
        busy = true
        generation++
        show(next, message)
        return generation
    }

    private fun cancelled(job: Int) = closed || job != generation || Thread.currentThread().isInterrupted
    private fun ui(job: Int, action: () -> Unit) { handler.post { if (!cancelled(job)) action() } }
    private fun fail(job: Int, error: Exception) = ui(job) {
        busy = false
        show("error", if (error is UpdateRejected) error.message.orEmpty() else "네트워크 연결을 확인하고 다시 시도해 주세요.")
    }
    private fun show(next: String, message: String, progress: Int? = null) {
        phase = next
        snapshot = state(next, message, progress)
        emit()
    }
    private fun state(next: String, message: String, progress: Int? = null) = JSONObject()
        .put("state", next).put("message", message).put("currentVersion", BuildConfig.VERSION_NAME)
        .put("latestVersion", release?.versionName ?: "").put("notes", release?.notes ?: "")
        .put("size", release?.size ?: 0).put("progress", progress ?: JSONObject.NULL)
        .put("hasUpdate", (release?.versionCode ?: 0) > BuildConfig.VERSION_CODE)

    private fun fetchRelease(job: Int): AppRelease {
        val request = Request.Builder().url(AppUpdatePolicy.MANIFEST_URL)
            .header("Accept", "application/json").header("Cache-Control", "no-cache").build()
        var text = ""
        execute(job, request) { response ->
            if (!response.isSuccessful) throw IOException("manifest HTTP ${response.code}")
            val body = response.body ?: throw IOException("empty manifest")
            if (body.contentLength() > AppUpdatePolicy.MAX_MANIFEST_BYTES) throw UpdateRejected("업데이트 정보가 올바르지 않아요.")
            body.byteStream().use { input ->
                val output = ByteArrayOutputStream()
                val buffer = ByteArray(4096)
                while (true) {
                    if (cancelled(job)) throw IOException("cancelled")
                    val read = input.read(buffer)
                    if (read == -1) break
                    if (output.size() + read > AppUpdatePolicy.MAX_MANIFEST_BYTES) throw UpdateRejected("업데이트 정보가 올바르지 않아요.")
                    output.write(buffer, 0, read)
                }
                text = output.toString("UTF-8")
            }
        }
        val parsed = AppUpdatePolicy.parse(text, context.packageName)
        AppUpdatePolicy.newer(parsed, BuildConfig.VERSION_CODE.toLong(), Build.VERSION.SDK_INT)
        if (cancelled(job)) throw IOException("cancelled")
        preferences.edit().putString("manifest", text).apply()
        return parsed
    }

    private fun execute(job: Int, request: Request, action: (okhttp3.Response) -> Unit) {
        if (cancelled(job)) throw IOException("cancelled")
        val requestCall = client.newCall(request)
        call = requestCall
        try {
            if (cancelled(job)) { requestCall.cancel(); throw IOException("cancelled") }
            requestCall.execute().use(action)
        } finally { if (call === requestCall) call = null }
    }

    @Suppress("DEPRECATION")
    private fun verifyArchive(file: File, selected: AppRelease) {
        val flags = if (Build.VERSION.SDK_INT >= 28) PackageManager.GET_SIGNING_CERTIFICATES else PackageManager.GET_SIGNATURES
        val installed = context.packageManager.getPackageInfo(context.packageName, flags)
        val archive = context.packageManager.getPackageArchiveInfo(file.absolutePath, flags)
            ?: throw UpdateRejected("유효한 Android 앱 파일이 아니에요.")
        fun version(info: PackageInfo) = if (Build.VERSION.SDK_INT >= 28) info.longVersionCode else info.versionCode.toLong()
        fun signers(info: PackageInfo): Set<String> {
            val signatures = if (Build.VERSION.SDK_INT >= 28) info.signingInfo?.apkContentsSigners else info.signatures
            return signatures.orEmpty().map { AppUpdatePolicy.hex(MessageDigest.getInstance("SHA-256").digest(it.toByteArray())) }.toSet()
        }
        AppUpdatePolicy.verifyIdentity(selected, version(installed), archive.packageName,
            version(archive), archive.versionName.orEmpty(), archive.applicationInfo?.minSdkVersion ?: -1,
            signers(installed), signers(archive), Build.VERSION.SDK_INT)
    }

    companion object {
        private val DISCARD = object : OutputStream() {
            override fun write(value: Int) {}
            override fun write(bytes: ByteArray, offset: Int, count: Int) {}
        }
    }
}
