<script setup>
import { ref, reactive, onMounted, onUnmounted, nextTick, watch } from "vue";
const pages = [
    "系统总览",
    "底盘调试",
    "视觉调试",
    "地图与导航",
    "巡检任务",
    "数据采集与媒体",
];
const page = ref(0),
    state = ref({}),
    error = ref(""),
    notice = ref(""),
    config = reactive({}),
    managed = ref({}),
    models = ref({}),
    media = ref({ files: [] }),
    mission = ref({}),
    map = ref(null),
    canvas = ref(null);
const options = reactive({
        algorithm: "gmapping",
        map: "",
        route: "",
        navigation_only: false,
    }),
    speeds = reactive({ x: 0.15, y: 0.15, z: 0.4 }),
    detection = reactive({
        model: "people_detector",
        mode: "onnx",
        confidence: 0.5,
        iou: 0.4,
    }),
    result = ref(null),
    live = ref(false),
    preview = ref(""),
    annotated = ref(""),
    topic = ref("/usb_cam/image_raw"),
    selection = ref([]),
    capture = reactive({ root: "", dataset: "default", interval: 0 }),
    poseTool = ref("goal"),
    angle = ref(0),
    savePath = ref("/home/mowen/maps/community_infra"),
    busy = ref(false);
let timers = [],
    socket,
    motionTimer,
    moving = false,
    inferenceRunning = false;
async function api(path, body) {
    const r = await fetch(
        "/api" + path,
        body === undefined
            ? {}
            : {
                  method: "POST",
                  headers: { "Content-Type": "application/json" },
                  body: JSON.stringify(body),
              },
    );
    if (!r.ok) {
        let e;
        try {
            e = await r.json();
        } catch {
            e = { detail: r.statusText };
        }
        throw Error(
            typeof e.detail === "string" ? e.detail : JSON.stringify(e.detail),
        );
    }
    return r.json();
}
async function act(fn) {
    error.value = "";
    try {
        return await fn();
    } catch (e) {
        error.value = e.message;
    }
}
async function refresh() {
    try {
        state.value = await api("/state");
        if (page.value === 3) {
            map.value = await api("/map");
            await nextTick();
            draw();
        }
    } catch (e) {
        state.value = { online: false };
        error.value = "后端连接中断：" + e.message;
        stop();
    }
}
async function service(name, action) {
    await act(async () => {
        await api(`/process/${name}/${action}`, options);
        notice.value = "操作已提交，启动结果请查看节点和日志";
        await processRefresh();
    });
}
async function processRefresh() {
    const p = await api("/processes");
    managed.value = p.processes;
    if (p.error) error.value = p.error;
}
async function mode(mode, locked) {
    stop();
    await act(() => api("/control", { mode, locked }));
    await refresh();
}
async function emergency() {
    stop();
    await act(async () => {
        notice.value = (await api("/estop", {})).message || "软件急停已触发";
    });
    await refresh();
}
function openSocket() {
    if (socket && socket.readyState <= 1) return;
    socket = new WebSocket(
        `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/api/control/ws`,
    );
    socket.onmessage = (e) => {
        error.value = JSON.parse(e.data).error;
        stop();
    };
    socket.onclose = () => {
        clearInterval(motionTimer);
        moving = false;
    };
}
function move(x, y, z, event) {
    event.preventDefault();
    if (
        event.button !== 0 ||
        state.value.estop ||
        state.value.mode !== "manual"
    )
        return;
    stop();
    openSocket();
    if (socket.readyState !== 1) {
        error.value = "控制连接尚未就绪，请重试";
        return;
    }
    moving = true;
    event.target.setPointerCapture(event.pointerId);
    const send = () => {
        if (socket.readyState === 1)
            socket.send(
                JSON.stringify({
                    x: x * speeds.x,
                    y: y * speeds.y,
                    z: z * speeds.z,
                }),
            );
    };
    send();
    motionTimer = setInterval(send, 100);
}
function stop() {
    moving = false;
    clearInterval(motionTimer);
    if (socket?.readyState === 1) socket.send(JSON.stringify({ stop: true }));
}
const directions = [
    [1, 1, 0, "↖"],
    [1, 0, 0, "↑"],
    [1, -1, 0, "↗"],
    [0, 1, 0, "←"],
    [0, 0, 0, "■"],
    [0, -1, 0, "→"],
    [-1, 1, 0, "↙"],
    [-1, 0, 0, "↓"],
    [-1, -1, 0, "↘"],
    [0, 0, 1, "↶"],
    [0, 0, -1, "↷"],
];
async function visionOnce() {
    if (inferenceRunning) return;
    inferenceRunning = true;
    await act(async () => {
        result.value = await api("/vision", detection);
        annotated.value = "/api/vision/image?t=" + Date.now();
    });
    inferenceRunning = false;
}
async function mediaRefresh() {
    media.value = await api("/media");
    if (!capture.root) capture.root = media.value.root;
}
async function zip() {
    await act(async () => {
        const r = await fetch("/api/media/export", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ files: selection.value }),
        });
        if (!r.ok) throw Error("导出失败");
        const url = URL.createObjectURL(await r.blob());
        const a = document.createElement("a");
        a.href = url;
        a.download = "media.zip";
        a.click();
        setTimeout(() => URL.revokeObjectURL(url), 1000);
    });
}
function draw() {
    const c = canvas.value,
        m = map.value;
    if (!c || !m) return;
    const ctx = c.getContext("2d"),
        w = m.width,
        h = m.height;
    c.width = w;
    c.height = h;
    const image = ctx.createImageData(w, h);
    m.data.forEach((v, i) => {
        const k = ((h - 1 - Math.floor(i / w)) * w + (i % w)) * 4;
        const color = v < 0 ? 75 : 255 - Math.round(v * 2.55);
        image.data.set([color, color, color, 255], k);
    });
    ctx.putImageData(image, 0, 0);
    const [ox, oy, a] = m.origin;
    const pixel = ([x, y]) => {
        const dx = x - ox,
            dy = y - oy;
        return [
            (Math.cos(a) * dx + Math.sin(a) * dy) / m.resolution,
            h - (-Math.sin(a) * dx + Math.cos(a) * dy) / m.resolution,
        ];
    };
    for (const [name, points] of Object.entries(state.value.paths || {})) {
        ctx.beginPath();
        ctx.strokeStyle = name.includes("local") ? "#ffbd69" : "#5ce1e6";
        ctx.lineWidth = 2;
        points.forEach((p, i) => {
            const q = pixel(p);
            i ? ctx.lineTo(...q) : ctx.moveTo(...q);
        });
        ctx.stroke();
    }
    ctx.fillStyle = "#ee7755";
    for (const p of state.value.scan || []) {
        const q = pixel(p);
        ctx.fillRect(q[0], q[1], 1, 1);
    }
    if (state.value.pose) {
        const p = state.value.pose,
            q = pixel(p);
        ctx.save();
        ctx.translate(...q);
        ctx.rotate(-(p[2] - a));
        ctx.fillStyle = "#29eaaa";
        ctx.beginPath();
        ctx.moveTo(10, 0);
        ctx.lineTo(-6, 5);
        ctx.lineTo(-6, -5);
        ctx.fill();
        ctx.restore();
    }
}
async function mapClick(e) {
    const m = map.value;
    if (!m) return;
    const box = canvas.value.getBoundingClientRect(),
        px = ((e.clientX - box.left) / box.width) * m.width,
        py = m.height - ((e.clientY - box.top) / box.height) * m.height;
    const [ox, oy, a] = m.origin,
        dx = px * m.resolution,
        dy = py * m.resolution;
    await act(() =>
        api("/pose/" + poseTool.value, {
            x: ox + Math.cos(a) * dx - Math.sin(a) * dy,
            y: oy + Math.sin(a) * dx + Math.cos(a) * dy,
            yaw: Number(angle.value),
        }),
    );
}
watch(page, () => {
    stop();
    live.value = false;
    nextTick(draw);
    if (page.value === 1) openSocket();
    if (page.value === 5) act(mediaRefresh);
});
watch(
    () => detection.model,
    (k) => {
        const m = models.value[k];
        if (m) {
            detection.confidence = m.conf_threshold;
            detection.iou = m.iou_threshold;
        }
    },
);
const keys = {
    q: [1, 1, 0],
    w: [1, 0, 0],
    e: [1, -1, 0],
    a: [0, 1, 0],
    d: [0, -1, 0],
    z: [-1, 1, 0],
    s: [-1, 0, 0],
    c: [-1, -1, 0],
    r: [0, 0, 1],
    f: [0, 0, -1],
};
function keydown(e) {
    if (
        page.value !== 1 ||
        e.repeat ||
        /INPUT|TEXTAREA|SELECT/.test(e.target.tagName)
    )
        return;
    const d = keys[e.key.toLowerCase()];
    if (!d) return;
    e.preventDefault();
    if (
        socket?.readyState !== 1 ||
        state.value.estop ||
        state.value.mode !== "manual"
    )
        return;
    stop();
    const send = () => {
        if (socket?.readyState === 1)
            socket.send(
                JSON.stringify({
                    x: d[0] * speeds.x,
                    y: d[1] * speeds.y,
                    z: d[2] * speeds.z,
                }),
            );
    };
    send();
    motionTimer = setInterval(send, 100);
}
function keyup(e) {
    if (keys[e.key.toLowerCase()]) stop();
}
function hidden() {
    if (document.hidden) stop();
}
onMounted(async () => {
    await act(async () => {
        Object.assign(config, await api("/config"));
        models.value = await api("/models");
        await mediaRefresh();
    });
    await refresh();
    timers.push(setInterval(refresh, 1000));
    timers.push(
        setInterval(async () => {
            if ([2, 5].includes(page.value) && state.value.camera_online)
                preview.value = "/api/image?t=" + Date.now();
            else preview.value = "";
            if (page.value === 2 && live.value) await visionOnce();
        }, 500),
    );
    timers.push(
        setInterval(() => {
            if (page.value === 0) act(processRefresh);
            if (page.value === 4)
                act(async () => (mission.value = await api("/mission")));
            if (page.value === 5) act(mediaRefresh);
        }, 3000),
    );
    window.addEventListener("keydown", keydown);
    window.addEventListener("keyup", keyup);
    window.addEventListener("blur", stop);
    document.addEventListener("visibilitychange", hidden);
    window.addEventListener("pointerup", stop);
    window.addEventListener("pointercancel", stop);
});
onUnmounted(() => {
    timers.forEach(clearInterval);
    stop();
    socket?.close();
    window.removeEventListener("keydown", keydown);
    window.removeEventListener("keyup", keyup);
    window.removeEventListener("blur", stop);
    document.removeEventListener("visibilitychange", hidden);
    window.removeEventListener("pointerup", stop);
    window.removeEventListener("pointercancel", stop);
});
</script>

<template>
    <div class="shell">
        <aside>
            <div class="brand">
                Smart Community<small>INFRA / 实车调试工作台</small>
            </div>
            <nav>
                <button
                    v-for="(name, i) in pages"
                    :class="{ active: page === i }"
                    @click="page = i"
                >
                    <span>0{{ i + 1 }}</span
                    >{{ name }}
                </button>
            </nav>
            <p class="hint">
                软件停车不能替代物理急停。<br />实车端须保留速度超时停车。
            </p>
        </aside>
        <main>
            <header>
                <div>
                    <h1>{{ pages[page] }}</h1>
                    <span :class="state.online ? 'online' : 'offline'"
                        >●
                        {{
                            state.online
                                ? "ROS Master 已连接"
                                : "ROS Master 离线"
                        }}</span
                    >
                </div>
                <div class="row">
                    <span
                        >{{ state.mode === "manual" ? "手动" : "自动" }} ·
                        {{ state.estop ? "控制锁定" : "控制已解锁" }}</span
                    ><button class="danger" @click="emergency">软件急停</button>
                </div>
            </header>
            <div v-if="error" class="alert" @click="error = ''">
                {{ error }} ×
            </div>
            <div v-if="notice" class="notice" @click="notice = ''">
                {{ notice }}
            </div>
            <section v-if="page === 0">
                <div class="grid">
                    <article>
                        <h2>网络配置</h2>
                        <label
                            >ROS Master URI<input
                                v-model="config.master_uri" /></label
                        ><label
                            >本机 ROS_IP<input v-model="config.ros_ip" /></label
                        ><button
                            @click="
                                act(
                                    async () =>
                                        (notice = (await api('/config', config))
                                            .message),
                                )
                            "
                        >
                            保存（重启后生效）</button
                        ><button @click="refresh">检查连接</button>
                        <p class="hint">
                            {{
                                state.error ||
                                "Master XML-RPC 可达；Topic 与 TF 请以下方实测为准"
                            }}
                            · SSH {{ config.ssh_host || "本机" }}
                        </p>
                    </article>
                    <article>
                        <h2>基础自检</h2>
                        <button @click="refresh">一键检查</button>
                        <div
                            v-for="t in [
                                '/odom_raw',
                                '/odom',
                                '/scan',
                                '/wit/imu',
                                'TF map→base_footprint',
                            ]"
                            class="statusrow"
                        >
                            <span>{{ t }}</span
                            ><b
                                :class="
                                    state.checks?.[t] ? 'online' : 'offline'
                                "
                                >{{
                                    state.checks?.[t] ? "正常" : "无新数据"
                                }}</b
                            >
                        </div>
                        <div class="statusrow">
                            相机
                            <b
                                :class="
                                    state.camera_online ? 'online' : 'offline'
                                "
                                >{{
                                    state.camera_online ? "正常" : "无新数据"
                                }}</b
                            >
                        </div>
                    </article>
                </div>
                <article>
                    <h2>服务与节点</h2>
                    <div
                        v-for="n in [
                            'hardware',
                            'mapping',
                            'navigation',
                            'services',
                            'mission',
                        ]"
                        class="statusrow"
                    >
                        <strong>{{ n }}</strong
                        ><span>{{
                            managed[n]?.running
                                ? "Infra 进程运行中"
                                : "未由 Infra 运行"
                        }}</span
                        ><button @click="service(n, 'start')">启动</button
                        ><button @click="service(n, 'stop')">停止</button>
                    </div>
                    <div class="tags">
                        <span v-for="n in state.nodes">{{ n }}</span>
                    </div>
                </article>
                <article>
                    <h2>ROS 实时日志</h2>
                    <pre class="logs">{{
                        (state.logs || [])
                            .map((l) => `[${l.level}] ${l.node}: ${l.text}`)
                            .join("\n")
                    }}</pre>
                </article>
            </section>
            <section v-if="page === 1">
                <article>
                    <h2>控制权限</h2>
                    <div class="row">
                        <button @click="mode('manual', true)">手动模式</button
                        ><button @click="mode('auto', true)">自动模式</button
                        ><button @click="mode(state.mode, false)">
                            解除软件急停</button
                        ><button @click="mode(state.mode, true)">锁定</button>
                    </div>
                    <p class="hint">
                        手动控制前停止导航、巡检及其他 /cmd_vel
                        发布者。按住移动，松开停车。键盘 QWE / A·D / ZSC
                        八方向，R/F 旋转；输入框聚焦时禁用快捷键。
                    </p>
                    <p>其他发布者：{{ state.conflicts?.join(", ") || "无" }}</p>
                </article>
                <div class="grid">
                    <article>
                        <h2>麦克纳姆控制</h2>
                        <div class="row">
                            <label v-for="k in ['x', 'y', 'z']"
                                >{{
                                    k === "z"
                                        ? "Yaw (rad/s)"
                                        : k.toUpperCase() + " (m/s)"
                                }}<input
                                    v-model.number="speeds[k]"
                                    type="number"
                                    min="0"
                                    :max="k === 'z' ? 1 : 0.5"
                                    step="0.05"
                            /></label>
                        </div>
                        <div class="pad">
                            <button
                                v-for="d in directions"
                                :disabled="
                                    state.estop || state.mode !== 'manual'
                                "
                                @pointerdown="move(...d.slice(0, 3), $event)"
                                @pointerup="stop"
                                @lostpointercapture="stop"
                            >
                                {{ d[3] }}
                            </button>
                        </div>
                    </article>
                    <article>
                        <h2>里程计与实时速度</h2>
                        <pre>{{ state.odom || "等待 /odom" }}</pre>
                        <svg viewBox="-5 -5 10 10" class="trajectory">
                            <polyline
                                :points="
                                    (state.trail || [])
                                        .map((p) => `${p[0]},${-p[1]}`)
                                        .join(' ')
                                "
                                fill="none"
                                stroke="#45dec3"
                                stroke-width="0.03"
                            /></svg
                        ><small>odom 坐标轨迹，显示范围 ±5 m</small>
                    </article>
                </div>
            </section>
            <section v-if="page === 2">
                <article>
                    <h2>模型与测试方式</h2>
                    <div class="row">
                        <select v-model="detection.model">
                            <option v-for="(_, name) in models">
                                {{ name }}
                            </option></select
                        ><select v-model="detection.mode">
                            <option value="onnx">独立 ONNX</option>
                            <option value="service">ROS 检测服务</option>
                            <option value="ocr">车牌 OCR</option></select
                        ><label
                            >Confidence<input
                                v-model.number="detection.confidence"
                                type="number"
                                min="0"
                                max="1"
                                step="0.05"
                                :disabled="detection.mode !== 'onnx'" /></label
                        ><label
                            >IoU<input
                                v-model.number="detection.iou"
                                type="number"
                                min="0"
                                max="1"
                                step="0.05"
                                :disabled="detection.mode !== 'onnx'"
                        /></label>
                    </div>
                    <p class="hint">
                        临时阈值只作用于独立 ONNX，不改动运行中 ROS
                        服务参数。ROS 服务须预先启动。
                    </p>
                    <button @click="visionOnce">测试当前帧</button
                    ><button @click="live = !live">
                        {{ live ? "停止连续测试" : "连续测试" }}</button
                    ><button @click="act(() => api('/vision/save', {}))">
                        保存原图及结果
                    </button>
                </article>
                <div class="grid">
                    <article>
                        <h2>原始图像</h2>
                        <img v-if="preview" :src="preview" />
                        <p v-else>相机离线</p>
                    </article>
                    <article>
                        <h2>识别画面</h2>
                        <img v-if="annotated" :src="annotated" />
                        <pre>{{ result || "等待测试" }}</pre>
                    </article>
                </div>
            </section>
            <section v-if="page === 3">
                <article>
                    <h2>建图与导航</h2>
                    <div class="row">
                        <select v-model="options.algorithm">
                            <option>gmapping</option>
                            <option>cartographer</option></select
                        ><button @click="service('mapping', 'start')">
                            启动建图</button
                        ><button @click="service('mapping', 'stop')">
                            停止建图</button
                        ><input
                            v-model="savePath"
                            placeholder="实车地图保存路径（不含扩展名）"
                        /><button
                            @click="
                                act(() => api('/map/save', { path: savePath }))
                            "
                        >
                            保存地图
                        </button>
                    </div>
                    <div class="row">
                        <input
                            v-model="options.map"
                            placeholder="实车地图 YAML 绝对路径，留空使用默认"
                        /><button @click="service('navigation', 'start')">
                            加载并启动导航</button
                        ><button @click="service('navigation', 'stop')">
                            停止导航
                        </button>
                    </div>
                    <div class="row">
                        <select v-model="poseTool">
                            <option value="goal">点击发送导航目标</option>
                            <option value="initial">
                                点击设置初始位姿
                            </option></select
                        ><label
                            >朝向 (rad)<input
                                v-model.number="angle"
                                type="number"
                                step="0.1" /></label
                        ><button
                            @click="
                                act(() => api('/pose/cancel', { x: 0, y: 0 }))
                            "
                        >
                            取消导航</button
                        ><button @click="mode('auto', false)">
                            自动模式并解锁
                        </button>
                    </div>
                    <p class="hint">
                        地图坐标系
                        map；橙色：雷达，青色：全局路径，黄色：局部路径，绿色：机器人。点击前确认模式与朝向。
                    </p>
                </article>
                <article>
                    <canvas
                        v-show="map"
                        ref="canvas"
                        @click="mapClick"
                    ></canvas>
                    <p v-if="!map">等待真实 /map 数据</p>
                </article>
            </section>
            <section v-if="page === 4">
                <article>
                    <h2>现有巡检程序</h2>
                    <div class="row">
                        <select v-model="options.navigation_only">
                            <option :value="false">完整巡检</option>
                            <option :value="true">仅导航</option></select
                        ><input
                            v-model="options.route"
                            placeholder="实车 route.yaml 路径，留空使用默认"
                        /><button @click="service('mission', 'start')">
                            启动任务</button
                        ><button @click="service('mission', 'stop')">
                            停止任务
                        </button>
                    </div>
                    <p class="hint">
                        使用 community_mission 原有状态机；通过 SSH
                        读取最近运行结果和日志。工作台退出后任务继续运行。
                    </p>
                </article>
                <article>
                    <h2>
                        {{ mission.result?.status || "暂无结果" }} ·
                        {{ mission.result?.state || "" }} · 当前航点
                        {{ mission.result?.current_waypoint || "—" }}
                    </h2>
                    <div class="tags">
                        <span v-for="p in mission.result?.waypoints"
                            >{{ p.id }} / {{ p.status }}</span
                        >
                    </div>
                    <pre>{{ mission.result || "等待真实结果 JSON" }}</pre>
                </article>
                <article>
                    <h2>任务日志</h2>
                    <pre class="logs">{{ mission.log }}</pre>
                </article>
            </section>
            <section v-if="page === 5">
                <article>
                    <h2>相机来源</h2>
                    <div class="row">
                        <input
                            v-model="topic"
                            list="image-topics"
                            placeholder="ROS Image Topic"
                        /><datalist id="image-topics">
                            <option
                                v-for="t in (state.topics || []).filter(
                                    (t) => t[1] === 'sensor_msgs/Image',
                                )"
                                :value="t[0]"
                            /></datalist
                        ><button @click="act(() => api('/camera', { topic }))">
                            切换来源
                        </button>
                    </div>
                    <p class="hint">
                        仅复用 ROS Image Topic；DJI / Orbbec
                        请先由已有相机节点发布图像。
                    </p>
                    <img v-if="preview" :src="preview" />
                    <p v-else>相机离线</p>
                </article>
                <article>
                    <h2>采集与录制</h2>
                    <div class="row">
                        <label>保存目录<input v-model="capture.root" /></label
                        ><label>数据集<input v-model="capture.dataset" /></label
                        ><label
                            >间隔秒（0 停止）<input
                                v-model.number="capture.interval"
                                type="number"
                                min="0"
                                step="0.2"
                        /></label>
                    </div>
                    <button
                        @click="
                            act(async () => {
                                await api('/media/settings', capture);
                                await mediaRefresh();
                            })
                        "
                    >
                        应用 / 定时采集</button
                    ><button @click="act(() => api('/media/photo', {}))">
                        拍照</button
                    ><button
                        @click="
                            act(async () => {
                                await api('/media/video/start', {});
                                await mediaRefresh();
                            })
                        "
                    >
                        开始录像</button
                    ><button
                        @click="
                            act(async () => {
                                await api('/media/video/stop', {});
                                await mediaRefresh();
                            })
                        "
                    >
                        停止录像
                    </button>
                    <p>
                        录像：{{ media.recording ? "进行中" : "已停止" }} ·
                        连续采集：{{ media.interval || "已停止" }}
                        {{ media.error }}
                    </p>
                </article>
                <article>
                    <h2>已保存媒体</h2>
                    <button @click="act(mediaRefresh)">刷新</button
                    ><button @click="zip" :disabled="!selection.length">
                        导出所选 ZIP
                    </button>
                    <div v-for="f in media.files" class="statusrow">
                        <input
                            type="checkbox"
                            v-model="selection"
                            :value="f"
                        /><a
                            :href="
                                '/api/media/file?path=' + encodeURIComponent(f)
                            "
                            target="_blank"
                            >{{ f }}</a
                        ><a
                            :href="
                                '/api/media/file?path=' + encodeURIComponent(f)
                            "
                            download
                            >下载</a
                        >
                    </div>
                </article>
            </section>
        </main>
    </div>
</template>
