/**
 * useJointChart — 关节角 Canvas 2D 曲线图 composable
 *
 * 替代旧版: vision_grasp/joint_chart.js (231行)
 *
 * 功能:
 *   - Canvas 2D 折线图（6 关节 × 最多 280 采样点）
 *   - Y 轴自适应缩放
 *   - 图例（颜色 → 关节名）
 *   - ResizeObserver 自适应
 *   - requestAnimationFrame 防抖重绘
 *
 * 用法:
 *   const { canvasRef, legendRef, pushSample, reset } = useJointChart()
 *   pushSample(['joint_1','joint_2',...], [0.1, 0.2, ...])
 */
const MAX_SAMPLES = 280
const LINE_COLORS = ['#2563eb', '#16a34a', '#d97706', '#db2777', '#7c3aed', '#0d9488']
const PAD = { L: 44, R: 8, T: 10, B: 22 }
const BG_COLOR = '#0f172a'
const BORDER_COLOR = '#334155'
const MUTED_COLOR = '#94a3b8'

export function useJointChart() {
  const canvasRef = ref<HTMLCanvasElement | null>(null)
  const legendRef = ref<HTMLElement | null>(null)

  let state = { names: [] as string[], series: [] as number[][] }
  let drawRaf: number | null = null
  let resizeObs: ResizeObserver | null = null

  function drawImmediate(): void {
    const canvas = canvasRef.value
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const rect = canvas.getBoundingClientRect()
    let cssW = Math.max(200, Math.floor(rect.width) || 640)
    let cssH = Math.max(160, Math.floor(rect.height) || 240)
    const dpr = (typeof window !== 'undefined' && window.devicePixelRatio) || 1

    canvas.style.width = `${cssW}px`
    canvas.style.height = `${cssH}px`
    canvas.width = Math.floor(cssW * dpr)
    canvas.height = Math.floor(cssH * dpr)
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0)

    const W = cssW; const H = cssH
    const plotW = W - PAD.L - PAD.R
    const plotH = H - PAD.T - PAD.B

    ctx.fillStyle = BG_COLOR
    ctx.fillRect(0, 0, W, H)
    ctx.strokeStyle = BORDER_COLOR
    ctx.lineWidth = 1
    ctx.strokeRect(0.5, 0.5, W - 1, H - 1)

    const series = state.series
    if (!series.some(arr => arr.length > 0)) return

    let yMin = Infinity; let yMax = -Infinity
    for (const arr of series) {
      for (const v of arr) {
        if (isFinite(v)) { if (v < yMin) yMin = v; if (v > yMax) yMax = v }
      }
    }
    if (!isFinite(yMin) || !isFinite(yMax)) return
    if (yMin === yMax) { yMin -= 1; yMax += 1 }
    const yPad = (yMax - yMin) * 0.08 || 0.05
    yMin -= yPad; yMax += yPad

    // 网格线
    ctx.strokeStyle = BORDER_COLOR
    ctx.globalAlpha = 0.35
    ctx.beginPath()
    for (let g = 0; g <= 4; g++) {
      const gy = PAD.T + (plotH * g) / 4
      ctx.moveTo(PAD.L, gy); ctx.lineTo(PAD.L + plotW, gy)
    }
    ctx.stroke()
    ctx.globalAlpha = 1

    // Y 轴刻度
    ctx.fillStyle = MUTED_COLOR
    ctx.font = '10px ui-monospace, monospace'
    ctx.textAlign = 'right'
    ctx.textBaseline = 'middle'
    for (let g = 0; g <= 4; g++) {
      const v = yMax - ((yMax - yMin) * g) / 4
      const gy = PAD.T + (plotH * g) / 4
      ctx.fillText(v.toFixed(3), PAD.L - 6, gy)
    }

    // 曲线
    for (let j = 0; j < series.length; j++) {
      const arr = series[j]
      if (arr.length === 0) continue
      const color = LINE_COLORS[j % LINE_COLORS.length]

      if (arr.length === 1 && isFinite(arr[0])) {
        const yNorm = (arr[0] - yMin) / (yMax - yMin)
        const y = PAD.T + plotH * (1 - yNorm)
        ctx.fillStyle = color
        ctx.beginPath(); ctx.arc(PAD.L + plotW / 2, y, 3.5, 0, Math.PI * 2); ctx.fill()
        continue
      }
      if (arr.length < 2) continue

      const denom = Math.max(1, arr.length - 1)
      ctx.strokeStyle = color
      ctx.lineWidth = 1.5
      ctx.beginPath()
      for (let t = 0; t < arr.length; t++) {
        const x = PAD.L + (t / denom) * plotW
        const yNorm = (arr[t] - yMin) / (yMax - yMin)
        const y = PAD.T + plotH * (1 - yNorm)
        if (t === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y)
      }
      ctx.stroke()
    }

    // X 轴标签
    ctx.fillStyle = MUTED_COLOR
    ctx.font = '10px system-ui, sans-serif'
    ctx.textAlign = 'center'
    ctx.textBaseline = 'top'
    ctx.fillText('时间 →', PAD.L + plotW / 2, H - PAD.B + 4)
  }

  function scheduleDraw(): void {
    if (drawRaf != null) return
    drawRaf = requestAnimationFrame(() => { drawRaf = null; drawImmediate() })
  }

  function updateLegend(): void {
    const leg = legendRef.value
    if (!leg) return
    leg.innerHTML = ''
    if (!state.names.length) { leg.setAttribute('aria-hidden', 'true'); return }
    state.names.forEach((n, i) => {
      const row = document.createElement('span')
      row.style.display = 'inline-flex'
      row.style.alignItems = 'center'
      row.style.gap = '4px'
      row.style.marginRight = '8px'
      row.style.fontSize = '11px'
      const sw = document.createElement('span')
      sw.style.display = 'inline-block'
      sw.style.width = '10px'; sw.style.height = '10px'
      sw.style.borderRadius = '2px'
      sw.style.background = LINE_COLORS[i % LINE_COLORS.length]
      row.appendChild(sw)
      const lab = document.createElement('span')
      lab.textContent = n; lab.style.color = '#94a3b8'
      row.appendChild(lab)
      leg.appendChild(row)
    })
    leg.setAttribute('aria-hidden', 'false')
  }

  function reset(): void {
    state = { names: [], series: [] }
    if (drawRaf != null) { cancelAnimationFrame(drawRaf); drawRaf = null }
    updateLegend()
    drawImmediate()
  }

  function pushSample(rawNames: string[], positions: number[]): void {
    const n = Math.max(rawNames.length, positions.length)
    if (n === 0) return
    const normNames = rawNames.map((rn, i) => rn || `joint_${i}`)
    const keyNew = normNames.join('\0')
    const keyOld = state.names.join('\0')
    if (state.series.length !== n || keyNew !== keyOld) {
      state.names = normNames.slice()
      state.series = Array.from({ length: n }, () => [])
    }
    for (let i = 0; i < n; i++) {
      const v = Number(positions[i])
      if (!isFinite(v)) continue
      const row = state.series[i]
      row.push(v)
      while (row.length > MAX_SAMPLES) row.shift()
    }
    updateLegend()
    scheduleDraw()
  }

  function observeResize(): void {
    const canvas = canvasRef.value
    if (!canvas) return
    if (typeof ResizeObserver !== 'undefined' && canvas.parentElement) {
      if (resizeObs) resizeObs.disconnect()
      resizeObs = new ResizeObserver(() => scheduleDraw())
      resizeObs.observe(canvas.parentElement)
    }
    scheduleDraw()
  }

  // 清理
  onUnmounted(() => {
    if (drawRaf != null) cancelAnimationFrame(drawRaf)
    if (resizeObs) resizeObs.disconnect()
  })

  return { canvasRef, legendRef, pushSample, reset, observeResize, scheduleDraw }
}
