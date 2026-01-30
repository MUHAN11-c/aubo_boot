// ============================================================
// 自动手眼标定功能扩展
// 此文件包含自动标定相关的所有功能
// 需要在 script_v2.js 加载后加载此文件
// ============================================================

// 固定的拍照位姿（写死）
const FIXED_PHOTO_POSE = {
    position: {
        x: 0.4682631492614746,
        y: -0.07431541383266449,
        z: 0.5182442665100098
    },
    orientation: {
        x: 0.7071067811865476,
        y: 0.7071067811865476,
        z: 0,
        w: 0
    }
};

// 自动标定全局状态（完全自动化模式）
let autoCalibState = {
    isRunning: false,
    isPaused: false,
    currentStep: 0,
    totalSteps: 0,              // 初始设置为0（无默认位姿）
    calibrationType: 'eye-in-hand',
    calibrationMethod: 'pose-based',  // 使用姿态法
    calibrationAlgorithm: 'custom',   // 'custom' 或 'opencv' - 标定算法选择
    opencvAlgorithm: 'TSAI',         // OpenCV 算法选择: 'TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS'
    // 自动化数据
    recordedPoses: [],           // 空数组，无默认位姿
    collectedCalibrationData: [], // 收集的标定数据（每个位姿的机器人位姿+标定板位姿）
    // 标定结果
    calibrationResult: null      // 存储最新的标定结果
};

// 初始化自动标定选项卡功能
function initAutoHandEyeCalibTab() {
    console.log('初始化自动手眼标定功能...');    // 初始化自动标定按钮
    initAutoCalibButtons();
    
    // 初始化标定类型切换
    initAutoCalibTypeChange();
    
    // 初始化智能状态卡片
    updateAutoStatusCard();
    
    // 更新UI显示（显示默认位姿）
    updateRecordedPosesDisplay();
    updateAutoCalibrationProgress();
    
    // 注意：图像缩放功能由 script_v2.js 的 initImageZoom() 统一初始化
    // 放大镜功能已从自动手眼标定选项卡中移除
    
    addLog('success', '✅ 自动标定系统初始化完成');
    addLog('info', '📍 固定拍照位姿已配置: X=0.468m, Y=-0.074m, Z=0.518m');
    addLog('info', '💡 提示: 请从 auto_hand_eye_poses_optimized.json 文件加载位姿数据');
}

// 初始化标定方法切换按钮
function initCalibrationMethodToggle() {
    const btnCustom = document.getElementById('btn-method-custom');
    const btnOpenCV = document.getElementById('btn-method-opencv');
    const opencvAlgorithmSetting = document.getElementById('opencv-algorithm-setting');
    const opencvAlgorithmSelect = document.getElementById('opencv-algorithm-select');
    
    // 初始化算法选择下拉框事件
    if (opencvAlgorithmSelect) {
        opencvAlgorithmSelect.addEventListener('change', function() {
            autoCalibState.opencvAlgorithm = this.value;
            const algorithmNames = {
                'TSAI': 'TSAI (经典方法，稳定可靠)',
                'PARK': 'PARK (Park & Martin)',
                'HORAUD': 'HORAUD (Horaud & Dornaika)',
                'ANDREFF': 'ANDREFF (Andreff et al.)',
                'DANIILIDIS': 'DANIILIDIS (Daniilidis)'
            };
            addLog('info', `🔄 已切换OpenCV算法: ${algorithmNames[this.value] || this.value}`);
        });
    }
    
    // 更新算法选择UI显示
    function updateAlgorithmSettingVisibility() {
        if (opencvAlgorithmSetting) {
            if (autoCalibState.calibrationAlgorithm === 'opencv') {
                opencvAlgorithmSetting.style.display = 'flex';
            } else {
                opencvAlgorithmSetting.style.display = 'none';
            }
        }
    }
    
    if (btnCustom && btnOpenCV) {
        // 点击自定义方法
        btnCustom.addEventListener('click', function() {
            autoCalibState.calibrationAlgorithm = 'custom';
            btnCustom.classList.add('active');
            btnCustom.style.background = 'rgba(255,255,255,0.3)';
            btnCustom.style.color = 'white';
            btnOpenCV.classList.remove('active');
            btnOpenCV.style.background = 'transparent';
            btnOpenCV.style.color = 'rgba(255,255,255,0.7)';
            addLog('info', '🔄 已切换标定算法: Custom模式 (AX=XB方法)');
            updateAlgorithmSettingVisibility();
            // 更新数据显示
            updateAutoCalibrationDataDisplay();
        });
        
        // 点击OpenCV方法
        btnOpenCV.addEventListener('click', function() {
            autoCalibState.calibrationAlgorithm = 'opencv';
            btnOpenCV.classList.add('active');
            btnOpenCV.style.background = 'rgba(255,255,255,0.3)';
            btnOpenCV.style.color = 'white';
            btnCustom.classList.remove('active');
            btnCustom.style.background = 'transparent';
            btnCustom.style.color = 'rgba(255,255,255,0.7)';
            const algorithmNames = {
                'TSAI': 'TSAI',
                'PARK': 'PARK',
                'HORAUD': 'HORAUD',
                'ANDREFF': 'ANDREFF',
                'DANIILIDIS': 'DANIILIDIS'
            };
            addLog('info', `🔄 已切换标定算法: OpenCV模式 (${algorithmNames[autoCalibState.opencvAlgorithm] || autoCalibState.opencvAlgorithm}方法)`);
            updateAlgorithmSettingVisibility();
            // 更新数据显示
            updateAutoCalibrationDataDisplay();
        });
    }
    
    // 初始化时更新显示状态
    updateAlgorithmSettingVisibility();
}

// 初始化自动标定按钮事件
function initAutoCalibButtons() {
    // 初始化标定方法切换（Custom/OpenCV）
    initCalibrationMethodToggle();
    
    // 位姿管理按钮：记录、清空、保存、加载、移动到第一个位姿、刷新图像
    const btnRecordRobotPose = document.getElementById('btn-auto-record-robot-pose');
    const btnClearMotionData = document.getElementById('btn-auto-clear-motion-data');
    const btnSaveAllPoses = document.getElementById('btn-auto-save-all-poses');
    const btnLoadAllPoses = document.getElementById('btn-auto-load-all-poses');
    const btnMoveToFirstPose = document.getElementById('btn-auto-move-to-first-pose');
    const btnRefreshImage = document.getElementById('btn-auto-refresh-image');
    
    if (btnRecordRobotPose) {
        btnRecordRobotPose.addEventListener('click', () => {
            handleRecordRobotPose();
        });
    }
    if (btnClearMotionData) {
        btnClearMotionData.addEventListener('click', () => {
            handleClearMotionData();
        });
    }
    if (btnSaveAllPoses) {
        btnSaveAllPoses.addEventListener('click', () => {
            handleSaveAllPosesToFile();
        });
    }
    if (btnLoadAllPoses) {
        btnLoadAllPoses.addEventListener('click', () => {
            handleLoadAllPosesFromFile();
        });
    }
    if (btnMoveToFirstPose) {
        btnMoveToFirstPose.addEventListener('click', () => {
            handleMoveToFirstPose();
        });
    }
    if (btnRefreshImage) {
        btnRefreshImage.addEventListener('click', () => {
            handleRefreshImage();
        });
    }
    
    // 标定执行按钮：开始、停止、保存标定结果
    const btnAutoStartCalib = document.getElementById('btn-auto-start-calibration');
    const btnAutoStopCalib = document.getElementById('btn-auto-stop-calibration');
    const btnAutoSaveCalib = document.getElementById('btn-auto-save-calibration');
    
    if (btnAutoStartCalib) {
        btnAutoStartCalib.addEventListener('click', () => {
            handleAutoCalibStart();
        });
    }
    if (btnAutoStopCalib) {
        btnAutoStopCalib.addEventListener('click', () => {
            handleAutoCalibStop();
        });
    }
    if (btnAutoSaveCalib) {
        btnAutoSaveCalib.addEventListener('click', () => handleSaveCalibrationForAuto());
    }
}

// ============= 自动标定功能 =============

// 记录机器人位姿（先显示图像预览，再记录位姿）
// 功能：采集当前图像并获取机器人当前位姿，保存到位姿列表
async function handleRecordRobotPose() {
    addLog('info', '📡 开始记录机器人位姿...');
    
    try {
        // 步骤1: 先采集并显示图像预览（确保能看到标定板）
        addLog('info', '   📷 步骤1/2: 采集图像预览...');
        
        const captureResponse = await fetch('/api/camera/capture', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'}
        });
        
        if (!captureResponse.ok) {
            throw new Error(`拍照失败：HTTP ${captureResponse.status}`);
        }
        
        const captureData = await captureResponse.json();
        if (!captureData.success) {
            throw new Error(`拍照失败：${captureData.error || '未知错误'}`);
        }
        
        addLog('success', '   ✅ 图像采集完成');
        await sleep(200);
        
        // 等待图像可用并显示
        let imageData = null;
        for (let attempt = 1; attempt <= 5; attempt++) {
            await sleep(200);
            try {
                const imgResponse = await fetch('/api/current_image');
                const imgData = await imgResponse.json();
                
                if (imgData.success && imgData.image) {
                    imageData = imgData;
                    break;
                }
            } catch (error) {
                console.error(`获取图像尝试${attempt}失败:`, error);
            }
        }
        
        if (!imageData) {
            throw new Error('获取图像超时，请重试');
        }
        
        // 显示图像预览
        const imgElement = document.getElementById('auto-hand-eye-calib-image');
        const placeholder = document.getElementById('auto-hand-eye-calib-image-container')?.querySelector('.image-placeholder');
        
        if (imgElement) {
            // 清除旧的事件处理器
            imgElement.onload = null;
            imgElement.onerror = null;
            
            // 设置新的事件处理器
            imgElement.onload = function() {
                this.classList.add('loaded');
                // 隐藏占位符
                if (placeholder) {
                    placeholder.style.display = 'none';
                }
                addLog('success', '   ✅ 图像预览已显示');
            };
            
            imgElement.onerror = function() {
                addLog('error', '   ❌ 图像加载失败，请检查相机连接');
            };
            
            // 设置图像源（base64数据不需要时间戳）
            imgElement.src = imageData.image;
        }
        
        // 步骤2: 获取机器人当前位姿（Base->Gripper变换）
        addLog('info', '   🤖 步骤2/2: 获取机器人当前位姿...');
        
        const robotResponse = await fetch('/api/robot_status');
        
        if (!robotResponse.ok) {
            throw new Error(`HTTP ${robotResponse.status}`);
        }
        
        const robotData = await robotResponse.json();
        
        if (!robotData.success || !robotData.is_online) {
            throw new Error('机器人未在线');
        }
        
        // 保存机器人位姿（深拷贝避免引用共享，单位：米）
        const recordedPose = {
            position: {
                x: robotData.cartesian_position.position.x,
                y: robotData.cartesian_position.position.y,
                z: robotData.cartesian_position.position.z
            },
            orientation: {
                x: robotData.cartesian_position.orientation.x,
                y: robotData.cartesian_position.orientation.y,
                z: robotData.cartesian_position.orientation.z,
                w: robotData.cartesian_position.orientation.w
            }
        };
        
        // 添加到记录列表
        if (!autoCalibState.recordedPoses) {
            autoCalibState.recordedPoses = [];
        }
        autoCalibState.recordedPoses.push(JSON.parse(JSON.stringify(recordedPose)));
        
        const poseCount = autoCalibState.recordedPoses.length;
        
        addLog('success', `✅ 位姿 #${poseCount} 记录成功`);
        addLog('info', `   📍 位置: X=${recordedPose.position.x.toFixed(3)}m, Y=${recordedPose.position.y.toFixed(3)}m, Z=${recordedPose.position.z.toFixed(3)}m`);
        addLog('info', `   🧭 姿态(四元数): X=${recordedPose.orientation.x.toFixed(6)}, Y=${recordedPose.orientation.y.toFixed(6)}, Z=${recordedPose.orientation.z.toFixed(6)}, W=${recordedPose.orientation.w.toFixed(6)}`);
        addLog('info', `💡 当前已记录 ${poseCount} 个位姿 (至少需要3个，建议5-8个以提高精度)`);
        
        // 更新进度条总数
        autoCalibState.totalSteps = poseCount;
        
        // 更新UI显示
        updateRecordedPosesDisplay();
        updateAutoStatusCard();
        
    } catch (error) {
        addLog('error', `❌ 记录位姿失败: ${error.message}`);
        addLog('info', '💡 提示: 请确保机器人已连接并处于在线状态');
        console.error('记录位姿失败:', error);
    }
}

// 注意：handleCaptureAndExtractBoard函数已移除
// 标定板位姿提取功能已集成到自动标定流程的captureImageAndExtractCorners函数中

// 清空记录的位姿数据
// 功能：清空所有记录的位姿和采集的标定数据，重置状态
function handleClearMotionData() {
    const count = autoCalibState.recordedPoses?.length || 0;
    
    // 完全清空所有位姿和采集数据，重置进度
    autoCalibState.recordedPoses = [];
    autoCalibState.collectedCalibrationData = [];
    autoCalibState.totalSteps = 0;
    autoCalibState.currentStep = 0;
    
    addLog('info', `🗑️ 已清除所有位姿数据 (共 ${count} 个)`);
    
    // 更新UI显示
    updateRecordedPosesDisplay();
    updateAutoCalibrationProgress();
    updateAutoStatusCard();
    updateAutoCalibrationDataDisplay(null);
}

// 在图像中心绘制十字标记
// 功能：在图像中心绘制绿色十字，用于辅助对齐标定板
function drawCrosshairOnImage(imgElementId) {
    const imgElement = document.getElementById(imgElementId);
    if (!imgElement || !imgElement.complete) {
        console.warn('图像未加载完成，无法绘制十字');
        return;
    }
    
    const container = imgElement.parentElement;
    if (!container) {
        console.warn('找不到图像容器');
        return;
    }
    
    // 确保容器是相对定位（用于canvas绝对定位）
    const computedStyle = window.getComputedStyle(container);
    if (computedStyle.position === 'static') {
        container.style.position = 'relative';
    }
    
    // 查找或创建canvas覆盖层（用于绘制十字）
    let canvas = container.querySelector('.crosshair-canvas');
    if (!canvas) {
        canvas = document.createElement('canvas');
        canvas.className = 'crosshair-canvas';
        canvas.style.position = 'absolute';
        canvas.style.pointerEvents = 'none'; // 不阻挡鼠标事件
        canvas.style.zIndex = '10';
        container.appendChild(canvas);
    }
    
    // 设置canvas尺寸为640x480（采集图像的实际尺寸）
    const IMAGE_WIDTH = 640;
    const IMAGE_HEIGHT = 480;
    
    // 获取图像元素的位置和尺寸
    const containerRect = container.getBoundingClientRect();
    const imgRect = imgElement.getBoundingClientRect();
    
    // 计算图像相对于容器的偏移
    const offsetLeft = imgRect.left - containerRect.left;
    const offsetTop = imgRect.top - containerRect.top;
    
    // 设置Canvas的位置和尺寸匹配图像
    canvas.style.left = offsetLeft + 'px';
    canvas.style.top = offsetTop + 'px';
    canvas.style.width = imgRect.width + 'px';
    canvas.style.height = imgRect.height + 'px';
    
    // Canvas内部分辨率设置为640x480
    canvas.width = IMAGE_WIDTH;
    canvas.height = IMAGE_HEIGHT;
    
    // 绘制十字
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // 使用640x480图像的中心点
    const centerX = IMAGE_WIDTH / 2;  // 320
    const centerY = IMAGE_HEIGHT / 2; // 240
    const crosshairSize = Math.min(IMAGE_WIDTH, IMAGE_HEIGHT) * 0.05; // 十字大小为图像尺寸的5% = 24像素
    
    // 设置绘制样式
    ctx.strokeStyle = '#00ff00'; // 绿色
    ctx.lineWidth = 2;
    ctx.shadowColor = 'rgba(0, 0, 0, 0.5)';
    ctx.shadowBlur = 3;
    
    // 绘制水平线
    ctx.beginPath();
    ctx.moveTo(centerX - crosshairSize, centerY);
    ctx.lineTo(centerX + crosshairSize, centerY);
    ctx.stroke();
    
    // 绘制垂直线
    ctx.beginPath();
    ctx.moveTo(centerX, centerY - crosshairSize);
    ctx.lineTo(centerX, centerY + crosshairSize);
    ctx.stroke();
    
    // 绘制中心圆点
    ctx.beginPath();
    ctx.arc(centerX, centerY, 3, 0, 2 * Math.PI);
    ctx.fillStyle = '#00ff00';
    ctx.fill();
    
    addLog('success', '✅ 已在图像中心绘制十字标记');
}

// 刷新图像（不运动机械臂）
// 功能：在当前位姿下重新采集并显示图像，不移动机器人
async function handleRefreshImage() {
    addLog('info', '🔄 开始刷新图像...');
    
    try {
        // 步骤1: 采集图像（不移动机器人）
        addLog('info', '📷 正在采集图像...');
        
        const captureResponse = await fetch('/api/camera/capture', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'}
        });
        
        if (!captureResponse.ok) {
            throw new Error(`拍照失败：HTTP ${captureResponse.status}`);
        }
        
        const captureData = await captureResponse.json();
        if (!captureData.success) {
            throw new Error(`拍照失败：${captureData.error || '未知错误'}`);
        }
        
        addLog('success', '✅ 图像采集完成');
        await sleep(200);
        
        // 步骤2: 获取并显示图像
        addLog('info', '🖼️ 正在加载图像...');
        
        let imageData = null;
        for (let attempt = 1; attempt <= 5; attempt++) {
            await sleep(200);
            try {
                const imgResponse = await fetch('/api/current_image');
                const imgData = await imgResponse.json();
                
                if (imgData.success && imgData.image) {
                    imageData = imgData;
                    break;
                }
            } catch (error) {
                console.error(`获取图像尝试${attempt}失败:`, error);
            }
        }
        
        if (!imageData) {
            throw new Error('获取图像超时');
        }
        
        // 步骤3: 显示图像
        const imgElement = document.getElementById('auto-hand-eye-calib-image');
        const placeholder = document.getElementById('auto-hand-eye-calib-image-container')?.querySelector('.image-placeholder');
        
        if (imgElement) {
            imgElement.onload = function() {
                this.classList.add('loaded');
                if (placeholder) {
                    placeholder.style.display = 'none';
                }
                addLog('success', '✅ 图像已刷新并显示');
            };
            
            imgElement.onerror = function() {
                addLog('error', '❌ 图像加载失败');
            };
            
            imgElement.src = imageData.image;
        }
        
        addLog('success', '✅ 图像刷新完成');
        
    } catch (error) {
        addLog('error', `❌ 刷新图像失败: ${error.message}`);
        addLog('info', '💡 提示: 请检查相机服务是否正常运行');
        console.error('刷新图像失败:', error);
    }
}

// 运动到拍照位姿
// 功能：移动机器人到固定的拍照位姿，然后采集图像并在中心绘制十字
async function handleMoveToFirstPose() {
    // 使用固定的拍照位姿（FIXED_PHOTO_POSE）
    const firstPose = FIXED_PHOTO_POSE;
    
    addLog('info', '🚀 开始移动机器人到拍照位姿...');
    addLog('info', `   📍 目标位置: X=${firstPose.position.x.toFixed(3)}m, Y=${firstPose.position.y.toFixed(3)}m, Z=${firstPose.position.z.toFixed(3)}m`);
    addLog('info', `   🧭 目标姿态(四元数): X=${firstPose.orientation.x.toFixed(4)}, Y=${firstPose.orientation.y.toFixed(4)}, Z=${firstPose.orientation.z.toFixed(4)}, W=${firstPose.orientation.w.toFixed(4)}`);
    
    try {
        // 步骤1: 使用moveRobotToPose函数运动到目标位姿
        // velocityFactor=0.3, accelerationFactor=0.3 确保安全平稳运动
        await moveRobotToPose(firstPose, false, 0.3, 0.3);
        
        addLog('success', '✅ 机器人已到达拍照位姿');
        
        // 步骤2: 采集图像
        addLog('info', '📷 正在采集图像...');
        
        const captureResponse = await fetch('/api/camera/capture', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'}
        });
        
        if (!captureResponse.ok) {
            throw new Error(`拍照失败：HTTP ${captureResponse.status}`);
        }
        
        const captureData = await captureResponse.json();
        if (!captureData.success) {
            throw new Error(`拍照失败：${captureData.error || '未知错误'}`);
        }
        
        addLog('success', '✅ 图像采集完成');
        await sleep(200);
        
        // 步骤3: 获取并显示图像
        addLog('info', '🖼️ 正在加载图像...');
        
        let imageData = null;
        for (let attempt = 1; attempt <= 5; attempt++) {
            await sleep(200);
            try {
                const imgResponse = await fetch('/api/current_image');
                const imgData = await imgResponse.json();
                
                if (imgData.success && imgData.image) {
                    imageData = imgData;
                    break;
                }
            } catch (error) {
                console.error(`获取图像尝试${attempt}失败:`, error);
            }
        }
        
        if (!imageData) {
            throw new Error('获取图像超时');
        }
        
        // 步骤4: 显示图像
        const imgElement = document.getElementById('auto-hand-eye-calib-image');
        const placeholder = document.getElementById('auto-hand-eye-calib-image-container')?.querySelector('.image-placeholder');
        
        if (imgElement) {
            imgElement.onload = function() {
                this.classList.add('loaded');
                if (placeholder) {
                    placeholder.style.display = 'none';
                }
                addLog('success', '✅ 图像已显示');
                
                // 步骤5: 在图像中心绘制十字
                setTimeout(() => {
                    drawCrosshairOnImage('auto-hand-eye-calib-image');
                }, 100); // 延迟100ms确保图像完全渲染
            };
            
            imgElement.onerror = function() {
                addLog('error', '❌ 图像加载失败');
            };
            
            imgElement.src = imageData.image;
        }
        
        addLog('success', '✅ 运动和拍照完成');
        addLog('info', '💡 提示: 现在可以开始记录其他位姿或开始自动标定流程');
        
    } catch (error) {
        addLog('error', `❌ 运动到拍照位姿失败: ${error.message}`);
        addLog('info', '💡 提示: 请确保机器人已连接并处于在线状态');
        console.error('运动到拍照位姿失败:', error);
    }
}

// 显示记录的位姿列表
// 功能：更新UI中显示的位姿列表和数量
function updateRecordedPosesDisplay() {
    const listDiv = document.getElementById('auto-motion-groups-list');
    const countSpan = document.getElementById('auto-motion-groups-count');
    
    // 更新位姿数量显示
    if (countSpan) {
        countSpan.textContent = autoCalibState.recordedPoses?.length || 0;
    }
    
    if (!listDiv) return;
    
    const poseCount = autoCalibState.recordedPoses?.length || 0;
    
    if (poseCount === 0) {
        listDiv.innerHTML = `
            <div style="padding: 10px; background: white; border-radius: 4px; text-align: center; color: #666; font-size: 13px;">
                暂无记录的位姿，请点击"记录机器人位姿"按钮
            </div>
        `;
    } else if (poseCount === 1) {
        // 只有默认位姿
        const pose = autoCalibState.recordedPoses[0];
        listDiv.innerHTML = `
            <div style="padding: 8px; background: white; border-radius: 4px; font-size: 12px; line-height: 1.4;">
                <div style="color: #1976d2; font-weight: bold; margin-bottom: 4px;">📍 位姿 #1 (默认拍照位姿)</div>
                <div style="color: #666;">X: ${pose.position.x.toFixed(3)}m, Y: ${pose.position.y.toFixed(3)}m, Z: ${pose.position.z.toFixed(3)}m</div>
            </div>
        `;
    } else {
        listDiv.innerHTML = `
            <div style="padding: 10px; background: white; border-radius: 4px; text-align: center; color: #666; font-size: 13px;">
                已记录 ${poseCount} 个位姿（包含1个默认拍照位姿）
            </div>
        `;
    }
}

// 更新自动标定进度条
// 功能：更新进度条显示当前标定进度和状态
function updateAutoCalibrationProgress() {
    const progressDiv = document.getElementById('auto-motion-progress');
    
    if (!progressDiv) return;
    
    // 获取当前进度状态
    const totalSteps = autoCalibState.totalSteps || 0;
    const currentStep = autoCalibState.currentStep || 0;
    const isRunning = autoCalibState.isRunning || false;
    
    if (totalSteps === 0) {
        progressDiv.innerHTML = '<span style="color: #666;">尚未记录位姿</span>';
        return;
    }
    
    const progressPercent = totalSteps > 0 ? (currentStep / totalSteps) * 100 : 0;
    
    let statusText = '';
    let statusColor = '#2196f3';
    
    if (isRunning) {
        statusText = `进行中: ${currentStep}/${totalSteps}`;
        statusColor = '#2196f3';
    } else if (currentStep === totalSteps && totalSteps > 0) {
        statusText = `已完成: ${totalSteps}/${totalSteps}`;
        statusColor = '#4caf50';
    } else {
        statusText = `已记录 ${totalSteps} 个位姿`;
        statusColor = '#4caf50';
    }
    
    progressDiv.innerHTML = `
        <div style="margin-bottom: 8px;">
            <strong style="color: ${statusColor};">
                ${isRunning ? '🔄' : '✅'} ${statusText}
            </strong>
        </div>
        ${isRunning ? `
            <div style="width: 100%; background: #e0e0e0; border-radius: 4px; height: 20px; overflow: hidden;">
                <div style="width: ${progressPercent}%; background: linear-gradient(90deg, #2196f3 0%, #21cbf3 100%); height: 100%; transition: width 0.3s ease; display: flex; align-items: center; justify-content: center; color: white; font-size: 10px; font-weight: bold;">
                    ${Math.round(progressPercent)}%
                </div>
            </div>
        ` : ''}
        ${totalSteps >= 3 ? `<div style="color: #4caf50; font-size: 10px; margin-top: 4px;">✅ 已满足最小要求（3个），可以开始自动标定</div>` : ''}
        ${totalSteps < 3 ? `<div style="color: #ff9800; font-size: 10px; margin-top: 4px;">⚠️ 至少需要3个位姿，当前 ${totalSteps} 个</div>` : ''}
    `;
}

// 更新运动数据进度显示（保留用于兼容，但使用新函数）
function updateMotionProgress() {
    updateRecordedPosesDisplay();
    updateAutoCalibrationProgress();
}

// 保存记录的位姿到文件（v4.0格式）- 保存到服务器
// 功能：将记录的位姿列表保存为JSON文件（先尝试保存到服务器，失败则下载到本地）
async function handleSaveAllPosesToFile() {
    // 检查位姿数量（至少需要3个）
    if (!autoCalibState.recordedPoses || autoCalibState.recordedPoses.length < 3) {
        addLog('warning', `⚠️ 位姿数据不足: 至少需要3个位姿，当前只有 ${autoCalibState.recordedPoses?.length || 0} 个`);
        addLog('info', '💡 提示: 请先记录足够的位姿数据后再开始自动标定');
        return;
    }
    
    const poseCount = autoCalibState.recordedPoses.length;
    
    // 构建v4.0格式的配置文件
    const config = {
        version: '4.0',
        calibrationType: 'eye-in-hand',
        calibrationMethod: 'pose-based-auto',
        savedAt: new Date().toISOString(),
        recordedPoses: autoCalibState.recordedPoses
    };
    
    try {
        addLog('info', '💾 正在保存位姿文件到服务器...');
        
        const response = await fetch('/api/poses/save', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(config)
        });
        
        const result = await response.json();
        
        if (result.success) {
            addLog('success', `💾 位姿文件已保存到服务器: ${result.filename}`);
            addLog('info', `   📍 保存路径: config/poses/${result.filename}`);
            showToast(`位姿文件已保存: ${result.filename}`, 'success');
        } else {
            addLog('error', `❌ 保存失败: ${result.error}`);
            showToast(`保存失败: ${result.error}`, 'error');
            // 如果服务器保存失败，回退到本地下载
            addLog('info', '💡 回退到本地下载...');
            const dataStr = JSON.stringify(config, null, 2);
            const blob = new Blob([dataStr], {type: 'application/json'});
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `auto_hand_eye_poses_${Date.now()}.json`;
            a.click();
            URL.revokeObjectURL(url);
            addLog('success', `💾 配置已下载到本地: ${poseCount} 个位姿数据 (v4.0格式)`);
        }
    } catch (error) {
        addLog('error', `❌ 保存失败: ${error.message}`);
        showToast(`保存失败: ${error.message}`, 'error');
        // 如果服务器保存失败，回退到本地下载
        addLog('info', '💡 回退到本地下载...');
        const dataStr = JSON.stringify(config, null, 2);
        const blob = new Blob([dataStr], {type: 'application/json'});
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `auto_hand_eye_poses_${Date.now()}.json`;
        a.click();
        URL.revokeObjectURL(url);
        addLog('success', `💾 配置已下载到本地: ${poseCount} 个位姿数据 (v4.0格式)`);
    }
}

// 从文件加载运动数据 - 支持从服务器列表选择或本地文件选择
// 功能：加载位姿配置文件，优先从服务器选择，失败则使用本地文件选择器
async function handleLoadAllPosesFromFile() {
    // 先尝试从服务器获取文件列表
    try {
        addLog('info', '📂 正在获取服务器上的位姿文件列表...');
        const listResponse = await fetch('/api/poses/list');
        const listResult = await listResponse.json();
        
        if (listResult.success && listResult.files && listResult.files.length > 0) {
            // 显示文件选择对话框（服务器文件列表）
            showPoseFileSelectionDialog(listResult.files);
            return;
        } else {
            addLog('info', '💡 服务器上没有位姿文件，使用本地文件选择');
        }
    } catch (error) {
        addLog('warning', `⚠️ 获取服务器文件列表失败: ${error.message}`);
        addLog('info', '💡 使用本地文件选择');
    }
    
    // 如果服务器没有文件或获取失败，使用本地文件选择
    loadPoseFileFromLocal();
}

// 显示位姿文件选择对话框
function showPoseFileSelectionDialog(files) {
    // 创建模态对话框
    const modal = document.createElement('div');
    modal.id = 'pose-file-selection-modal';
    modal.style.cssText = `
        position: fixed;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        background: rgba(0, 0, 0, 0.5);
        display: flex;
        align-items: center;
        justify-content: center;
        z-index: 10000;
    `;
    
    const content = document.createElement('div');
    content.style.cssText = `
        background: white;
        border-radius: 8px;
        padding: 20px;
        max-width: 600px;
        width: 90%;
        max-height: 80vh;
        overflow-y: auto;
        box-shadow: 0 4px 20px rgba(0,0,0,0.3);
    `;
    
    content.innerHTML = `
        <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 15px; border-bottom: 2px solid #667eea; padding-bottom: 10px;">
            <h3 style="margin: 0; color: #667eea; font-size: 18px;">📂 选择位姿文件</h3>
            <button id="close-pose-file-modal" style="background: none; border: none; font-size: 24px; cursor: pointer; color: #999; padding: 0; width: 30px; height: 30px;">&times;</button>
        </div>
        <div style="margin-bottom: 15px;">
            <div style="background: #f5f5f5; padding: 10px; border-radius: 4px; margin-bottom: 10px;">
                <strong>服务器上的位姿文件 (${files.length} 个):</strong>
            </div>
            <div id="server-file-list" style="max-height: 300px; overflow-y: auto; border: 1px solid #e0e0e0; border-radius: 4px;">
                ${files.map((file, index) => `
                    <div class="pose-file-item" data-filename="${file.filename}" style="padding: 12px; border-bottom: 1px solid #f0f0f0; cursor: pointer; transition: background 0.2s; ${index === files.length - 1 ? 'border-bottom: none;' : ''}">
                        <div style="font-weight: 500; color: #333; margin-bottom: 4px;">${file.filename}</div>
                        <div style="font-size: 12px; color: #999;">
                            <span>修改时间: ${file.modified}</span>
                            <span style="margin-left: 15px;">大小: ${(file.size / 1024).toFixed(2)} KB</span>
                        </div>
                    </div>
                `).join('')}
            </div>
        </div>
        <div style="display: flex; gap: 10px; justify-content: flex-end; border-top: 1px solid #e0e0e0; padding-top: 15px;">
            <button id="load-from-local-btn" style="padding: 8px 16px; background: #2196f3; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px;">📁 从本地文件加载</button>
            <button id="cancel-pose-file-modal" style="padding: 8px 16px; background: #999; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 14px;">取消</button>
        </div>
    `;
    
    modal.appendChild(content);
    document.body.appendChild(modal);
    
    // 关闭对话框
    const closeModal = () => {
        document.body.removeChild(modal);
    };
    
    document.getElementById('close-pose-file-modal').onclick = closeModal;
    document.getElementById('cancel-pose-file-modal').onclick = closeModal;
    modal.onclick = (e) => {
        if (e.target === modal) closeModal();
    };
    
    // 文件项点击事件
    const fileItems = content.querySelectorAll('.pose-file-item');
    fileItems.forEach(item => {
        item.onmouseenter = () => {
            item.style.background = '#f0f7ff';
        };
        item.onmouseleave = () => {
            item.style.background = 'white';
        };
        item.onclick = async () => {
            const filename = item.dataset.filename;
            closeModal();
            await loadPoseFileFromServer(filename);
        };
    });
    
    // 从本地文件加载按钮
    document.getElementById('load-from-local-btn').onclick = () => {
        closeModal();
        loadPoseFileFromLocal();
    };
}

// 从服务器加载位姿文件
async function loadPoseFileFromServer(filename) {
    try {
        addLog('info', `📂 正在从服务器加载位姿文件: ${filename}`);
        
        const response = await fetch('/api/poses/load', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ filename: filename })
        });
        
        const result = await response.json();
        
        if (result.success && result.data) {
            processLoadedPoseData(result.data);
        } else {
            addLog('error', `❌ 加载失败: ${result.error}`);
            showToast(`加载失败: ${result.error}`, 'error');
        }
    } catch (error) {
        addLog('error', `❌ 加载失败: ${error.message}`);
        showToast(`加载失败: ${error.message}`, 'error');
    }
}

// 从本地文件加载位姿文件
function loadPoseFileFromLocal() {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    
    input.onchange = function(e) {
        const file = e.target.files[0];
        if (!file) return;
        
        const reader = new FileReader();
        reader.onload = function(event) {
            try {
                const config = JSON.parse(event.target.result);
                processLoadedPoseData(config);
            } catch (error) {
                addLog('error', `❌ 解析文件失败：${error.message}`);
                showToast(`解析文件失败: ${error.message}`, 'error');
            }
        };
        reader.readAsText(file);
    };
    
    input.click();
}

// 处理加载的位姿数据（通用函数）
// 功能：验证并加载位姿配置文件，更新全局状态
function processLoadedPoseData(config) {
    // 检查版本和数据结构（仅支持v4.0格式）
    if (config.version === '4.0' && config.recordedPoses) {
        // 新版本：记录的位姿列表（至少需要3个位姿）
        if (!Array.isArray(config.recordedPoses) || config.recordedPoses.length < 3) {
            addLog('warning', `⚠️ 配置文件中的位姿数据不足: 至少需要3个，当前只有 ${config.recordedPoses?.length || 0} 个`);
            showToast(`位姿数据不足: 需要至少3个位姿`, 'warning');
            return;
        }
        
        // 更新全局状态：位姿列表、总步数，清空已采集数据
        autoCalibState.recordedPoses = config.recordedPoses;
        autoCalibState.totalSteps = config.recordedPoses.length;
        autoCalibState.collectedCalibrationData = [];
        
        addLog('success', `✅ 配置已加载: ${config.recordedPoses.length} 个位姿数据`);
        addLog('info', '💡 提示: 现在可以开始自动标定流程');
        showToast(`成功加载 ${config.recordedPoses.length} 个位姿`, 'success');
        
    } else {
        addLog('error', '❌ 文件格式错误：不支持的版本或缺少必要字段');
        addLog('info', '💡 提示: 请使用v4.0格式的配置文件（包含recordedPoses字段）');
        showToast('文件格式错误：需要v4.0格式', 'error');
        return;
    }
    
    // 更新UI显示
    updateRecordedPosesDisplay();
    updateAutoCalibrationProgress();
    updateAutoStatusCard();
}

// 注意：旧的角点法相关函数已移除，当前只使用recordedPoses流程

// ============= 自动标定核心功能 =============

// 等待机器人运动完成

// 移动机器人到指定位姿
// 功能：调用后端API移动机器人到目标位姿，等待移动完成
// 参数：targetPose-目标位姿, useJoints-是否使用关节空间, velocityFactor-速度因子, accelerationFactor-加速度因子
async function moveRobotToPose(targetPose, useJoints = false, velocityFactor = 0.5, accelerationFactor = 0.5) {
    try {
        addLog('info', `🤖 正在移动机器人到位姿...`);
        addLog('info', `   位置: X=${targetPose.position.x.toFixed(3)}m, Y=${targetPose.position.y.toFixed(3)}m, Z=${targetPose.position.z.toFixed(3)}m`);
        if (targetPose.orientation) {
            addLog('info', `   姿态(四元数): x=${targetPose.orientation.x.toFixed(4)}, y=${targetPose.orientation.y.toFixed(4)}, z=${targetPose.orientation.z.toFixed(4)}, w=${targetPose.orientation.w.toFixed(4)}`);
        } else {
            addLog('warning', `   ⚠️  警告：位姿数据缺少orientation（旋转信息）！`);
        }
        
        // 调用后端API移动机器人
        const response = await fetch('/api/robot/move_to_pose', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
                target_pose: targetPose,
                use_joints: useJoints,
                velocity_factor: velocityFactor,
                acceleration_factor: accelerationFactor
            })
        });
        
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        
        const result = await response.json();
        
        if (!result.success) {
            throw new Error(result.error || '移动失败');
        }
        
        addLog('success', `✅ 机器人移动指令已发送`);
        
        // 注意：/api/robot/move_to_pose 是同步API，会等待机器人移动完成才返回
        // 因此不需要再次等待，只需要短暂延迟确保机器人完全稳定
        await sleep(100); // 短暂延迟确保机器人完全稳定
        
        addLog('success', `✅ 机器人已到达目标位姿`);
        
        // 异步检查机器人移动后的实际位姿（不阻塞主流程）
        fetch('/api/robot_status').then(robotStatusResponse => {
            if (robotStatusResponse.ok) {
                return robotStatusResponse.json();
            }
        }).then(robotStatus => {
            if (robotStatus && robotStatus.success && robotStatus.is_online) {
                const actualOri = robotStatus.cartesian_position.orientation;
                const targetQuat = targetPose.orientation;
                if (targetQuat && actualOri) {
                    // 简化的角度差计算（使用四元数点积）
                    const dot = targetQuat.x * actualOri.x + targetQuat.y * actualOri.y + 
                               targetQuat.z * actualOri.z + targetQuat.w * actualOri.w;
                    const angleDiff = 2 * Math.acos(Math.min(Math.abs(dot), 1.0)) * 180 / Math.PI;
                    
                    if (angleDiff > 5) {
                        addLog('warning', `   ⚠️  旋转角度差异较大: ${angleDiff.toFixed(2)}°，可能机器人未按目标旋转`);
                    }
                }
            }
        }).catch(() => {
            // 忽略错误
        });
        
        return true;
        
    } catch (error) {
        addLog('error', `❌ 移动机器人失败：${error.message}`);
        throw error;
    }
}

// 在当前位置采集图像和角点
// 功能：在当前位姿下拍照、提取标定板位姿、获取机器人位姿，返回完整的标定数据
// 参数：poseIndex-位姿索引
async function captureImageAndExtractCorners(poseIndex) {
    try {
        // 步骤1: 拍照（触发相机采集图像）
        addLog('info', `   📷 触发相机拍照...`);
        const captureResponse = await fetch('/api/camera/capture', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'}
        });
        
        if (!captureResponse.ok) {
            throw new Error(`拍照失败：HTTP ${captureResponse.status}`);
        }
        
        const captureData = await captureResponse.json();
        if (!captureData.success) {
            throw new Error(`拍照失败：${captureData.error || '未知错误'}`);
        }
        
        addLog('success', '   ✅ 拍照完成');
        await sleep(200);  // 优化：从500ms减少到200ms
        
        // 步骤2: 等待图像可用（轮询获取图像，最多5次）
        let imageData = null;
        for (let attempt = 1; attempt <= 5; attempt++) {  // 优化：从10次减少到5次
            await sleep(200);  // 优化：从500ms减少到200ms
            try {
                const imgResponse = await fetch('/api/current_image');
                const imgData = await imgResponse.json();
                
                if (imgData.success && imgData.image) {
                    imageData = imgData;
                    break;
                }
            } catch (error) {
                console.error(`获取图像尝试${attempt}失败:`, error);
            }
        }
        
        if (!imageData) {
            throw new Error('获取图像超时');
        }
        
        // 显示图像
        const imgElement = document.getElementById('auto-hand-eye-calib-image');
        const placeholder = document.getElementById('auto-hand-eye-calib-image-container')?.querySelector('.image-placeholder');
        
        if (imgElement) {
            imgElement.onload = null;
            imgElement.onerror = null;
            imgElement.onload = function() {
                this.classList.add('loaded');
                if (placeholder) {
                    placeholder.style.display = 'none';
                }
            };
            imgElement.src = imageData.image;
        }
        
        // 步骤3: 提取标定板位姿（使用solvePnP计算Board->Camera变换）
        addLog('info', `   🔍 提取标定板位姿...`);
        const squareSize = parseFloat(document.getElementById('auto-checkerboard-size').value) || 20;
        const boardPoseResponse = await fetch('/api/camera/get_board_pose', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({square_size: squareSize})
        });
        
        if (!boardPoseResponse.ok) {
            throw new Error(`提取标定板位姿失败：HTTP ${boardPoseResponse.status}`);
        }
        
        const boardPoseData = await boardPoseResponse.json();
        if (!boardPoseData.success) {
            throw new Error(`提取标定板位姿失败：${boardPoseData.error || '未知错误'}`);
        }
        
        // 显示带角点的图像（用于可视化验证）
        if (boardPoseData.image_with_corners) {
            if (imgElement) {
                imgElement.onload = null;
                imgElement.onerror = null;
                imgElement.onload = function() {
                    this.classList.add('loaded');
                    if (placeholder) {
                        placeholder.style.display = 'none';
                    }
                };
                imgElement.src = boardPoseData.image_with_corners;
            }
        }
        
        addLog('success', `   ✅ 标定板位姿提取完成`);
        addLog('info', `      位置: X=${boardPoseData.position.x.toFixed(2)}, Y=${boardPoseData.position.y.toFixed(2)}, Z=${boardPoseData.position.z.toFixed(2)}`);
        if (boardPoseData.corners_count) {
            addLog('info', `      检测到 ${boardPoseData.corners_count} 个角点`);
        }
        
        // 步骤4: 获取当前机器人位姿（运动后的实际位姿，Base->Gripper）
        const robotResponse = await fetch('/api/robot_status');
        if (!robotResponse.ok) {
            throw new Error('获取机器人位姿失败');
        }
        
        const robotData = await robotResponse.json();
        if (!robotData.success || !robotData.is_online) {
            throw new Error('机器人未在线');
        }
        
        // 返回采集的数据（包含机器人位姿、标定板位姿、角点信息）
        return {
            pose_index: poseIndex,
            robot_pose: {
                position: {
                    x: robotData.cartesian_position.position.x,
                    y: robotData.cartesian_position.position.y,
                    z: robotData.cartesian_position.position.z
                },
                orientation: {
                    x: robotData.cartesian_position.orientation.x,
                    y: robotData.cartesian_position.orientation.y,
                    z: robotData.cartesian_position.orientation.z,
                    w: robotData.cartesian_position.orientation.w
                }
            },
            board_pose: {
                position: boardPoseData.position,
                orientation: boardPoseData.orientation,
                reprojection_error: boardPoseData.reprojection_error,  // 平均重投影误差（像素）
                max_reprojection_error: boardPoseData.max_reprojection_error,  // 最大重投影误差（像素）
                estimated_z_mm: boardPoseData.estimated_z_mm,  // solvePnP估计的z值（毫米）
                depth_avg_mm: boardPoseData.depth_avg_mm,  // 深度相机平均深度（毫米）
                depth_std_mm: boardPoseData.depth_std_mm,  // 深度标准差（毫米）
                depth_error_mm: boardPoseData.depth_error_mm,  // 深度误差（毫米）
                depth_error_pct: boardPoseData.depth_error_pct,  // 深度误差百分比
                depth_quality_ok: boardPoseData.depth_quality_ok,  // 深度数据质量标志
                rvec: boardPoseData.rvec,  // 旋转向量（用于OpenCV模式）
                tvec: boardPoseData.tvec   // 平移向量（用于OpenCV模式）
            },
            corners: boardPoseData.corners || [],  // 角点像素坐标
            corners_3d: boardPoseData.corners_3d || [],  // 角点相机坐标（包含所有角点，带深度验证信息）
            corners_3d_filtered: boardPoseData.corners_3d_filtered || []  // 滤波后的角点（用于手眼标定）
        };
        
    } catch (error) {
        addLog('error', `❌ 采集图像和角点失败：${error.message}`);
        throw error;
    }
}

// 更新自动标定按钮状态
// 功能：根据标定运行状态启用/禁用开始和停止按钮
function updateAutoCalibButtonsState(isRunning) {
    const btnAutoStartCalib = document.getElementById('btn-auto-start-calibration');
    const btnAutoStopCalib = document.getElementById('btn-auto-stop-calibration');
    
    // 运行时禁用开始按钮，启用停止按钮
    if (btnAutoStartCalib) {
        btnAutoStartCalib.disabled = isRunning;
        btnAutoStartCalib.style.opacity = isRunning ? '0.5' : '1';
    }
    if (btnAutoStopCalib) {
        btnAutoStopCalib.disabled = !isRunning;
        btnAutoStopCalib.style.opacity = isRunning ? '1' : '0.5';
    }
}

// 停止自动标定
// 功能：停止正在运行的自动标定流程，重置状态
function handleAutoCalibStop() {
    if (!autoCalibState.isRunning) {
        addLog('warning', '⚠️ 自动标定未在运行');
        return;
    }
    
    addLog('warning', '🛑 正在停止自动标定...');
    autoCalibState.isRunning = false;  // 设置停止标志
    updateAutoCalibrationProgress();
    
    // 更新按钮状态（启用开始按钮，禁用停止按钮）
    updateAutoCalibButtonsState(false);
    
    addLog('warning', '⚠️ 自动标定已停止');
}

// 开始自动标定（完全自动化流程）
// 功能：自动移动到每个记录的位姿，采集图像和标定板位姿，最后执行标定计算
async function handleAutoCalibStart() {
    try {
        // 检查是否已在运行
        if (autoCalibState.isRunning) {
            addLog('warning', '⚠️ 自动标定正在进行中');
            return;
        }
        
        // 检查记录的位姿数量（v4.0格式，至少需要3个）
        if (!autoCalibState.recordedPoses || autoCalibState.recordedPoses.length < 3) {
            addLog('error', `❌ 位姿数据不足，至少需要3个位姿，当前只有${autoCalibState.recordedPoses?.length || 0}个`);
            addLog('info', '💡 请先点击"记录机器人位姿"记录多个位姿（建议5-8个）');
            return;
        }
        
        // 检查相机参数是否已加载
        if (!cameraParamsLoaded) {
            addLog('warning', '⚠️ 请先加载相机参数（系统将自动从ROS2话题获取）');
            await new Promise(resolve => setTimeout(resolve, 1000));  // 优化：从2000ms减少到1000ms
            if (!cameraParamsLoaded) {
                addLog('error', '❌ 未能从ROS2话题获取相机参数，请检查相机是否在线');
                return;
            }
        }
        
        // 初始化状态：设置运行标志、重置进度、清空采集数据
        autoCalibState.isRunning = true;
        autoCalibState.currentStep = 0;
        autoCalibState.totalSteps = autoCalibState.recordedPoses.length;
        autoCalibState.collectedCalibrationData = [];
        updateAutoCalibrationDataDisplay(null);  // 清空数据显示
        
        // 更新按钮状态
        updateAutoCalibButtonsState(true);
        
        addLog('info', '════════════════════════════════════════════');
        addLog('info', `🚀 开始完全自动手眼标定`);
        addLog('info', `📋 位姿数量: ${autoCalibState.totalSteps}个`);
        addLog('info', `📋 标定方法: 姿态法（AX=XB）`);
        addLog('info', '════════════════════════════════════════════');
        
        // 清空collect_data目录
        addLog('info', '🗑️  清空collect_data目录...');
        try {
            const clearResponse = await fetch('/api/hand_eye/clear_collect_data', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'}
            });
            const clearData = await clearResponse.json();
            if (clearData.success) {
                addLog('success', `✅ ${clearData.message}`);
            } else {
                addLog('warning', `⚠️ 清空目录失败: ${clearData.error || '未知错误'}`);
            }
        } catch (error) {
            addLog('warning', `⚠️ 清空目录时出错: ${error.message}`);
        }
        
        // 更新进度条
        updateAutoCalibrationProgress();
        
        // 依次处理每个位姿（循环遍历所有记录的位姿）
        for (let i = 0; i < autoCalibState.recordedPoses.length && autoCalibState.isRunning; i++) {
            const poseIndex = i + 1;
            const targetPose = autoCalibState.recordedPoses[i];
            
            // 更新当前进度
            autoCalibState.currentStep = poseIndex;
            updateAutoCalibrationProgress();
            
            addLog('info', `\n📍 [${poseIndex}/${autoCalibState.totalSteps}] 处理位姿 #${poseIndex}`);
            addLog('info', `   目标位置: X=${targetPose.position.x.toFixed(3)}m, Y=${targetPose.position.y.toFixed(3)}m, Z=${targetPose.position.z.toFixed(3)}m`);
            
            try {
                // 检查是否已停止（在循环开始前检查）
                if (!autoCalibState.isRunning) {
                    addLog('warning', '⚠️ 自动标定已停止');
                    break;
                }
                
                // 步骤1: 移动机器人到目标位姿（使用较低的速度和加速度确保稳定）
                addLog('info', `   📍 [${poseIndex}/${autoCalibState.totalSteps}] 步骤1/3: 移动机器人到目标位姿...`);
                await moveRobotToPose(targetPose, false, 0.2, 0.1);
                
                // 检查是否已停止（在移动后）
                if (!autoCalibState.isRunning) {
                    addLog('warning', '⚠️ 自动标定已停止');
                    break;
                }
                
                // #region agent log H4 - 验证实际到达位姿与目标位姿的差异
                try {
                    const robotStatusResp = await fetch('/api/robot_status');
                    if (robotStatusResp.ok) {
                        const robotStatusData = await robotStatusResp.json();
                        if (robotStatusData.success && robotStatusData.is_online) {
                            const actualPose = robotStatusData.cartesian_position;
                            const posError = Math.sqrt(
                                Math.pow(actualPose.position.x - targetPose.position.x, 2) +
                                Math.pow(actualPose.position.y - targetPose.position.y, 2) +
                                Math.pow(actualPose.position.z - targetPose.position.z, 2)
                            ) * 1000.0; // 转换为mm
                            
                            fetch('http://127.0.0.1:7247/ingest/e9d26850-3ab7-4f99-a006-3c0f52f46c82', {
                                method: 'POST',
                                headers: {'Content-Type': 'application/json'},
                                body: JSON.stringify({
                                    location: 'script_v2_auto_calib_addon.js:1002',
                                    message: `位姿#${poseIndex}到达后实际位姿`,
                                    data: {
                                        pose_index: poseIndex,
                                        target_pose: {x: targetPose.position.x, y: targetPose.position.y, z: targetPose.position.z},
                                        actual_pose: {x: actualPose.position.x, y: actualPose.position.y, z: actualPose.position.z},
                                        position_error_mm: posError
                                    },
                                    timestamp: Date.now(),
                                    sessionId: 'debug-session',
                                    runId: 'pose-debug',
                                    hypothesisId: 'H4'
                                })
                            }).catch(() => {});
                        }
                    }
                } catch (e) {}
                // #endregion
                
                // OpenCV模式：位姿到位后延迟3秒，确保机器人稳定（避免振动影响）
                if (autoCalibState.calibrationAlgorithm === 'opencv') {
                    addLog('info', `   ⏱️  [${poseIndex}/${autoCalibState.totalSteps}] OpenCV模式：位姿到位后等待3秒，确保机器人稳定...`);
                    await sleep(3000);  // 延时3秒
                }
                
                // 步骤2: 采集图像和角点（拍照、提取标定板位姿、获取机器人位姿）
                addLog('info', `   📷 [${poseIndex}/${autoCalibState.totalSteps}] 步骤2/3: 采集图像和提取角点...`);
                const calibrationData = await captureImageAndExtractCorners(poseIndex);
                
                // 检查是否已停止（在采集图像后）
                if (!autoCalibState.isRunning) {
                    addLog('warning', '⚠️ 自动标定已停止');
                    break;
                }
                
                // 步骤3: 保存图像和位姿到collect_data目录（用于离线验证）
                addLog('info', `   💾 [${poseIndex}/${autoCalibState.totalSteps}] 步骤3/3: 保存图像和位姿数据...`);
                try {
                    // 获取square_size参数（与步骤2保持一致）
                    const squareSize = parseFloat(document.getElementById('auto-checkerboard-size').value) || 20;
                    const saveResponse = await fetch('/api/hand_eye/save_image_and_pose', {
                        method: 'POST',
                        headers: {'Content-Type': 'application/json'},
                        body: JSON.stringify({
                            pose_index: i,  // 使用从0开始的索引，与图像命名一致（images0.jpg, images1.jpg...）
                            robot_pose: calibrationData.robot_pose,
                            square_size: squareSize  // 传递square_size参数，与get_board_pose保持一致
                        })
                    });
                    const saveData = await saveResponse.json();
                    if (saveData.success) {
                        addLog('success', `   ✅ ${saveData.message}`);
                    } else {
                        addLog('warning', `   ⚠️ 保存失败: ${saveData.error || '未知错误'}`);
                    }
                } catch (error) {
                    addLog('warning', `   ⚠️ 保存时出错: ${error.message}`);
                }
                
                // 检查是否已停止（在保存后）
                if (!autoCalibState.isRunning) {
                    addLog('warning', '⚠️ 自动标定已停止');
                    break;
                }
                
                // 步骤4: 保存采集的数据到内存（用于后续标定计算）
                autoCalibState.collectedCalibrationData.push(calibrationData);
                
                // 更新数据显示（每个位姿到位后更新一次，实时显示采集进度）
                updateAutoCalibrationDataDisplay();
                
                // 显示重投影误差信息（评估标定板位姿计算质量）
                const lastData = autoCalibState.collectedCalibrationData[autoCalibState.collectedCalibrationData.length - 1];
                let logMsg = `✅ 位姿 #${poseIndex} 数据采集完成`;
                if (lastData?.board_pose?.reprojection_error !== undefined) {
                    const reprojError = lastData.board_pose.reprojection_error;
                    const maxReprojError = lastData.board_pose.max_reprojection_error || reprojError;
                    const cornerCount = lastData.corners?.length || 0;
                    logMsg += ` | 检测到 ${cornerCount} 个角点`;
                    if (reprojError < 1.0) {
                        logMsg += ` | 重投影误差: ${reprojError.toFixed(3)} px (优秀)`;
                    } else if (reprojError < 2.0) {
                        logMsg += ` | 重投影误差: ${reprojError.toFixed(3)} px (良好)`;
                    } else {
                        logMsg += ` | 重投影误差: ${reprojError.toFixed(3)} px (需改进)`;
                    }
                }
                addLog('success', logMsg);
                
                // OpenCV模式：数据采集完成后延时2秒，确保数据稳定后再移动到下一个位姿
                if (autoCalibState.calibrationAlgorithm === 'opencv') {
                    addLog('info', `   ⏱️  [${poseIndex}/${autoCalibState.totalSteps}] OpenCV模式：数据采集完成，等待2秒后移动到下一个位姿...`);
                    await sleep(2000);  // 延时2秒
                }
                
            } catch (error) {
                addLog('error', `❌ 位姿 #${poseIndex} 处理失败: ${error.message}`);
                addLog('warning', `⚠️ 跳过当前位姿，继续处理下一个位姿`);
                continue;
            }
        }
        
        if (!autoCalibState.isRunning) {
            addLog('warning', '⚠️ 自动标定已停止');
            return;
        }
        
        // 检查采集的数据是否足够（至少需要2组数据才能计算相对运动）
        const collectedCount = autoCalibState.collectedCalibrationData.length;
        if (collectedCount < 2) {
            addLog('error', `❌ 有效数据不足: 至少需要2组运动数据，当前只有 ${collectedCount} 组`);
            addLog('info', '💡 提示: 请确保已成功采集足够的位姿数据');
            autoCalibState.isRunning = false;
            return;
        }
        
        // 再次检查是否已停止（在执行标定计算前）
        if (!autoCalibState.isRunning) {
            addLog('warning', '⚠️ 自动标定已停止，跳过标定计算');
            return;
        }
        
        // 步骤4: 执行标定计算（使用采集的所有位姿数据）
        addLog('info', '');
        addLog('info', '═══════════════════════════════════════════');
        addLog('info', `📊 开始标定计算 (使用 ${collectedCount} 组运动数据)...`);
        
        await performAutoCalibrationFromCollectedData();
        
        // 检查是否已停止（在执行标定计算后）
        if (!autoCalibState.isRunning) {
            addLog('warning', '⚠️ 自动标定已停止');
            return;
        }
        
        autoCalibState.isRunning = false;
        updateAutoCalibrationProgress();
        
        // 更新按钮状态
        updateAutoCalibButtonsState(false);
        
        addLog('success', '🎉 自动标定流程完成！');
        addLog('info', '💡 提示: 可以查看标定结果并保存到配置文件');
        
    } catch (error) {
        addLog('error', `❌ 自动标定失败: ${error.message}`);
        addLog('info', '💡 提示: 请检查数据质量和系统连接状态');
        console.error('自动标定失败:', error);
        autoCalibState.isRunning = false;
        updateAutoCalibrationProgress();
        
        // 更新按钮状态
        updateAutoCalibButtonsState(false);
    }
}

// 使用采集的数据执行标定计算
// 功能：将采集的数据格式化为后端API需要的格式，调用标定API，保存结果
async function performAutoCalibrationFromCollectedData() {
    try {
        // 直接使用位姿列表格式，每个元素包含robot_pose和board_pose
        // 将前端数据格式转换为后端API需要的格式
        const posesList = autoCalibState.collectedCalibrationData.map(poseData => ({
            robot_pose: {
                robot_pos_x: poseData.robot_pose.position.x,
                robot_pos_y: poseData.robot_pose.position.y,
                robot_pos_z: poseData.robot_pose.position.z,
                robot_ori_x: poseData.robot_pose.orientation.x,
                robot_ori_y: poseData.robot_pose.orientation.y,
                robot_ori_z: poseData.robot_pose.orientation.z,
                robot_ori_w: poseData.robot_pose.orientation.w
            },
            board_pose: {
                position: poseData.board_pose.position,
                orientation: poseData.board_pose.orientation,
                reprojection_error: poseData.board_pose.reprojection_error,
                rvec: poseData.board_pose.rvec,  // OpenCV模式需要
                tvec: poseData.board_pose.tvec,  // OpenCV模式需要
                corners_3d_filtered: poseData.corners_3d_filtered || []
            }
        }));
        
        // 验证数据数量（至少需要3个位姿）
        if (posesList.length < 3) {
            throw new Error(`位姿数据不足，至少需要3个位姿，当前只有${posesList.length}个`);
        }
        
        // 构建标定请求数据（包含标定类型、方法、算法选择）
        const calibData = {
            calibration_type: 'eye-in-hand',
            calibration_method: 'pose-based',
            method: autoCalibState.calibrationAlgorithm || 'custom',  // 'custom' 或 'opencv'
            poses_list: posesList
        };
        
        // 如果是 OpenCV 模式，添加算法选择参数
        if (autoCalibState.calibrationAlgorithm === 'opencv') {
            calibData.opencv_algorithm = autoCalibState.opencvAlgorithm || 'TSAI';
        }
        
        addLog('info', `📊 提交标定数据: ${posesList.length} 个位姿`);
        
        const response = await fetch('/api/hand_eye/calibrate', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(calibData)
        });
        
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        
        const result = await response.json();
        
        if (result.success) {
            // 保存标定结果到全局状态（用于后续显示和保存）
            autoCalibState.calibrationResult = result;
            
            // 更新数据显示（不清空，保持所有位姿数据）
            updateAutoCalibrationDataDisplay();
            
            addLog('success', `✅ 标定计算成功！`);
            addLog('info', `📊 标定方法: ${result.method || 'Pose-Based (AX=XB)'}`);
            
            // 显示误差统计信息（AX=XB约束验证误差）
            if (result.evaluation) {
                addLog('info', `📊 标定误差统计:`);
                addLog('info', `   平均误差: ${result.evaluation.mean_error.toFixed(3)} mm`);
                addLog('info', `   最大误差: ${result.evaluation.max_error.toFixed(3)} mm`);
                addLog('info', `   最小误差: ${result.evaluation.min_error.toFixed(3)} mm`);
                addLog('info', `   标准差: ${result.evaluation.std_error.toFixed(3)} mm`);
                
                // 添加误差质量评估（根据平均误差评估标定质量）
                const meanError = result.evaluation.mean_error;
                if (meanError < 5.0) {
                    addLog('success', `   ✅ 标定精度: 优秀 (平均误差 < 5mm)`);
                } else if (meanError < 10.0) {
                    addLog('info', `   ⚠️ 标定精度: 良好 (平均误差 < 10mm)`);
                } else {
                    addLog('warning', `   ⚠️ 标定精度: 需改进 (平均误差 >= 10mm)`);
                }
            }
            
            updateAutoStatusCard();  // 更新状态卡片显示
        } else {
            addLog('error', `❌ 标定失败: ${result.error || '未知错误'}`);
            if (result.error_type) {
                addLog('info', `   错误类型: ${result.error_type}`);
            }
        }
        
    } catch (error) {
        addLog('error', `❌ 标定计算失败: ${error.message}`);
        addLog('info', '💡 提示: 请检查数据质量和网络连接');
        throw error;
    }
}

// ============= 代码清理说明 =============
// 以下旧功能已移除，当前只使用recordedPoses流程：
// - 旧的角点法流程（executeAutoCalibration, performAutoCalibration, handleStartCalibrationForAuto）
// - 旧的手动操作功能（handleCaptureImageForAuto, handleExtractCornersForAuto, handleSavePoseDataForAuto, handleLoadPoseDataForAuto）
// - 旧的角点质量评估（evaluateCornerQuality, selectBestCornerObservations）
// - 旧的UI更新函数（updateAutoCalibButtons, updateAutoStatus, updateAutoProgress, updateAutoCurrentStep, updateAutoPoseCount, updateAutoPoseListTable）
// - 旧的暂停/继续/停止功能（handleAutoCalibPause, handleAutoCalibResume, handleAutoCalibStop）
// 当前流程：handleAutoCalibStart -> performAutoCalibrationFromCollectedData

// 注意：以下旧的手动操作函数已完全移除：
// - handleCaptureImageForAuto
// - handleExtractCornersForAuto
// - handleSavePoseDataForAuto
// - handleLoadPoseDataForAuto
// - handleStartCalibrationForAuto
// 当前流程使用完全自动化的recordedPoses流程

// 辅助函数：计算旋转矩阵的欧拉角（ZYX顺序）
function rotationMatrixToEulerAngles(R) {
    const sy = Math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]);
    const singular = sy < 1e-6;
    
    let x, y, z;
    if (!singular) {
        x = Math.atan2(R[2][1], R[2][2]);
        y = Math.atan2(-R[2][0], sy);
        z = Math.atan2(R[1][0], R[0][0]);
    } else {
        x = Math.atan2(-R[1][2], R[1][1]);
        y = Math.atan2(-R[2][0], sy);
        z = 0;
    }
    
    return {
        roll: x * 180 / Math.PI,
        pitch: y * 180 / Math.PI,
        yaw: z * 180 / Math.PI
    };
}

// 辅助函数：计算旋转轴角
function rotationMatrixToAxisAngle(R) {
    const trace = R[0][0] + R[1][1] + R[2][2];
    const angle = Math.acos(Math.max(-1, Math.min(1, (trace - 1) / 2)));
    
    if (angle < 1e-6) {
        return { axis: [1, 0, 0], angle: 0 };
    }
    
    const axis = [
        R[2][1] - R[1][2],
        R[0][2] - R[2][0],
        R[1][0] - R[0][1]
    ];
    const norm = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    
    return {
        axis: [axis[0] / norm, axis[1] / norm, axis[2] / norm],
        angle: angle * 180 / Math.PI
    };
}

// 辅助函数：计算向量模长
function vectorMagnitude(v) {
    return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 辅助函数：计算误差分位数
function calculatePercentiles(errors) {
    const sorted = [...errors].sort((a, b) => a - b);
    return {
        p25: sorted[Math.floor(sorted.length * 0.25)],
        p50: sorted[Math.floor(sorted.length * 0.50)],
        p75: sorted[Math.floor(sorted.length * 0.75)],
        p90: sorted[Math.floor(sorted.length * 0.90)],
        p95: sorted[Math.floor(sorted.length * 0.95)]
    };
}

// 更新自动标定数据显示（表格形式，每个位姿的输入输出数据）
// 功能：以表格形式显示所有采集的位姿数据，根据算法模式显示不同的列
function updateAutoCalibrationDataDisplay() {
    const displayDiv = document.getElementById('auto-calibration-data-display');
    if (!displayDiv) return;
    
    // 获取已收集的数据列表（每个位姿的机器人位姿+标定板位姿）
    const collectedData = autoCalibState.collectedCalibrationData || [];
    
    // 获取当前选择的标定算法（custom或opencv）
    const algorithm = autoCalibState.calibrationAlgorithm || 'custom';
    
    if (collectedData.length === 0) {
        // 从标定结果中获取方法名称，如果没有则使用默认
        let methodName = algorithm === 'opencv' ? `OpenCV (${autoCalibState.opencvAlgorithm || 'TSAI'})` : 'Custom (AX=XB)';
        if (autoCalibState.calibrationResult && autoCalibState.calibrationResult.method) {
            methodName = autoCalibState.calibrationResult.method;
        }
        displayDiv.innerHTML = `<div style="text-align: center; color: #999; font-size: 15px; padding: 40px 20px;">
            🤖 等待自动标定开始，数据将显示在这里...<br>
            <span style="font-size: 13px; color: #bbb; margin-top: 8px; display: block;">当前模式: ${methodName}</span>
        </div>`;
        return;
    }
    
    // 构建表格HTML
    let html = '<style>';
    html += '.data-table { width: 100%; border-collapse: collapse; font-size: 13px; }';
    html += '.data-table thead { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; position: sticky; top: 0; z-index: 10; }';
    html += '.data-table th { padding: 12px 8px; text-align: center; font-weight: 600; border: 1px solid rgba(255,255,255,0.2); }';
    html += '.data-table td { padding: 10px 8px; text-align: center; border: 1px solid #dee2e6; background: white; }';
    html += '.data-table tbody tr:nth-child(even) { background: #f8f9fa; }';
    html += '.data-table tbody tr:hover { background: #e7f3ff; }';
    html += '.data-value { font-family: monospace; font-weight: 500; }';
    html += '.pose-number { font-weight: 600; color: #667eea; }';
    html += '</style>';
    
    // 显示当前模式提示
    // 从标定结果中获取方法名称，如果没有则使用默认
    let methodName = algorithm === 'opencv' ? `OpenCV (${autoCalibState.opencvAlgorithm || 'TSAI'})` : 'Custom (AX=XB)';
    if (autoCalibState.calibrationResult && autoCalibState.calibrationResult.method) {
        methodName = autoCalibState.calibrationResult.method;
    }
    html += `<div style="margin-bottom: 10px; padding: 8px 12px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; border-radius: 6px; font-size: 14px; font-weight: 500;">
        当前标定模式: ${methodName}
    </div>`;
    
    html += '<div style="overflow-x: auto;">';
    html += '<table class="data-table">';
    
    // 根据标定算法选择显示不同的表头
    html += '<thead>';
    html += '<tr>';
    html += '<th style="min-width: 60px;">位姿</th>';
    html += '<th style="min-width: 80px;">角点数</th>';
    
    if (algorithm === 'opencv') {
        // OpenCV模式：显示OpenCV相关的数据
        html += '<th style="min-width: 100px;">标定板 X (mm)</th>';
        html += '<th style="min-width: 100px;">标定板 Y (mm)</th>';
        html += '<th style="min-width: 100px;">标定板 Z (mm)</th>';
        html += '<th style="min-width: 120px;">机器人位姿 X (m)</th>';
        html += '<th style="min-width: 120px;">机器人位姿 Y (m)</th>';
        html += '<th style="min-width: 120px;">机器人位姿 Z (m)</th>';
        html += '<th style="min-width: 120px;">机器人姿态 Qx</th>';
        html += '<th style="min-width: 120px;">机器人姿态 Qy</th>';
        html += '<th style="min-width: 120px;">机器人姿态 Qz</th>';
        html += '<th style="min-width: 120px;">机器人姿态 Qw</th>';
        html += '<th style="min-width: 100px;">重投影误差 (px)</th>';
    } else {
        // Custom模式：显示详细的数据
        html += '<th style="min-width: 100px;">标定板 X (mm)</th>';
        html += '<th style="min-width: 100px;">标定板 Y (mm)</th>';
        html += '<th style="min-width: 100px;">标定板 Z (mm)</th>';
        html += '<th style="min-width: 120px;">估计Z (mm)</th>';
        html += '<th style="min-width: 120px;">深度Z (mm)</th>';
        html += '<th style="min-width: 120px;">机械臂位姿 X (mm)</th>';
        html += '<th style="min-width: 120px;">机械臂位姿 Y (mm)</th>';
        html += '<th style="min-width: 120px;">机械臂位姿 Z (mm)</th>';
        html += '<th style="min-width: 120px;">重投影误差 (px)</th>';
        html += '<th style="min-width: 120px;">最大误差 (px)</th>';
        html += '<th style="min-width: 100px;">深度质量</th>';
    }
    html += '</tr>';
    html += '</thead>';
    
    // 表格内容
    html += '<tbody>';
    collectedData.forEach((data, index) => {
        html += '<tr>';
        html += `<td class="pose-number">#${data.pose_index || (index + 1)}</td>`;
        html += `<td>${data.corners?.length || 0}</td>`;
        
        // 标定板位姿
        if (data.board_pose) {
            html += `<td class="data-value">${data.board_pose.position.x.toFixed(3)}</td>`;
            html += `<td class="data-value">${data.board_pose.position.y.toFixed(3)}</td>`;
            html += `<td class="data-value">${data.board_pose.position.z.toFixed(3)}</td>`;
            
            if (algorithm === 'opencv') {
                // OpenCV模式：显示机器人位姿（单位：米）和姿态（四元数）
                if (data.robot_pose && data.robot_pose.position) {
                    html += `<td class="data-value">${data.robot_pose.position.x.toFixed(6)}</td>`;
                    html += `<td class="data-value">${data.robot_pose.position.y.toFixed(6)}</td>`;
                    html += `<td class="data-value">${data.robot_pose.position.z.toFixed(6)}</td>`;
                } else {
                    html += '<td>-</td><td>-</td><td>-</td>';
                }
                
                // 机器人姿态（四元数）
                if (data.robot_pose && data.robot_pose.orientation) {
                    html += `<td class="data-value">${data.robot_pose.orientation.x.toFixed(6)}</td>`;
                    html += `<td class="data-value">${data.robot_pose.orientation.y.toFixed(6)}</td>`;
                    html += `<td class="data-value">${data.robot_pose.orientation.z.toFixed(6)}</td>`;
                    html += `<td class="data-value">${data.robot_pose.orientation.w.toFixed(6)}</td>`;
                } else {
                    html += '<td>-</td><td>-</td><td>-</td><td>-</td>';
                }
                
                // 重投影误差
                if (data.board_pose.reprojection_error !== undefined) {
                    const reprojError = data.board_pose.reprojection_error;
                    const reprojColor = reprojError < 1.0 ? '#28a745' : reprojError < 2.0 ? '#ffc107' : '#dc3545';
                    html += `<td class="data-value" style="color: ${reprojColor}; font-weight: bold;">${reprojError.toFixed(3)}</td>`;
                } else {
                    html += '<td>-</td>';
                }
            } else {
                // Custom模式：显示详细数据
                // 估计Z值（solvePnP估计的z值）
                if (data.board_pose.estimated_z_mm !== undefined && data.board_pose.estimated_z_mm !== null) {
                    html += `<td class="data-value">${data.board_pose.estimated_z_mm.toFixed(3)}</td>`;
                } else {
                    html += '<td>-</td>';
                }
                
                // 深度Z值（深度相机平均深度）
                if (data.board_pose.depth_avg_mm !== undefined && data.board_pose.depth_avg_mm !== null) {
                    html += `<td class="data-value">${data.board_pose.depth_avg_mm.toFixed(3)}</td>`;
                } else {
                    html += '<td>-</td>';
                }
                
                // 机械臂位姿 X (转换为毫米)
                if (data.robot_pose && data.robot_pose.position && data.robot_pose.position.x !== undefined) {
                    const robotX = data.robot_pose.position.x * 1000; // 米转毫米
                    html += `<td class="data-value">${robotX.toFixed(3)}</td>`;
                } else {
                    html += '<td>-</td>';
                }
                
                // 机械臂位姿 Y (转换为毫米)
                if (data.robot_pose && data.robot_pose.position && data.robot_pose.position.y !== undefined) {
                    const robotY = data.robot_pose.position.y * 1000; // 米转毫米
                    html += `<td class="data-value">${robotY.toFixed(3)}</td>`;
                } else {
                    html += '<td>-</td>';
                }
                
                // 机械臂位姿 Z (转换为毫米)
                if (data.robot_pose && data.robot_pose.position && data.robot_pose.position.z !== undefined) {
                    const robotZ = data.robot_pose.position.z * 1000; // 米转毫米
                    html += `<td class="data-value">${robotZ.toFixed(3)}</td>`;
                } else {
                    html += '<td>-</td>';
                }
                
                // 重投影误差
                if (data.board_pose.reprojection_error !== undefined) {
                    const reprojError = data.board_pose.reprojection_error;
                    const reprojColor = reprojError < 1.0 ? '#28a745' : reprojError < 2.0 ? '#ffc107' : '#dc3545';
                    html += `<td class="data-value" style="color: ${reprojColor}; font-weight: bold;">${reprojError.toFixed(3)}</td>`;
                } else {
                    html += '<td>-</td>';
                }
                
                // 最大重投影误差
                if (data.board_pose.max_reprojection_error !== undefined) {
                    const maxReprojError = data.board_pose.max_reprojection_error;
                    const maxReprojColor = maxReprojError < 1.0 ? '#28a745' : maxReprojError < 2.0 ? '#ffc107' : '#dc3545';
                    html += `<td class="data-value" style="color: ${maxReprojColor};">${maxReprojError.toFixed(3)}</td>`;
                } else {
                    html += '<td>-</td>';
                }
                
                // 深度质量
                if (data.board_pose.depth_quality_ok !== undefined) {
                    if (data.board_pose.depth_quality_ok) {
                        html += '<td class="data-value" style="color: #28a745; font-weight: bold;">✅ 良好</td>';
                    } else {
                        html += '<td class="data-value" style="color: #dc3545; font-weight: bold;">❌ 异常</td>';
                    }
                } else {
                    html += '<td>-</td>';
                }
            }
        } else {
            // 没有标定板数据，填充空单元格
            const emptyCellCount = algorithm === 'opencv' ? 10 : 10;
            html += '<td>-</td>'.repeat(emptyCellCount);
        }
        
        html += '</tr>';
    });
    html += '</tbody>';
    html += '</table>';
    html += '</div>';
    
    displayDiv.innerHTML = html;
}

// 更新标定结果显示（保留用于最终标定结果）
function updateCalibrationResultDisplay() {
    // 此函数已废弃，不再使用
    return;
    
    // 构建结果显示HTML
    let html = '<style>';
    html += '.calib-result-card { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 16px; border-radius: 10px; margin-bottom: 16px; }';
    html += '.calib-info-card { background: #f8f9fa; border: 1px solid #dee2e6; border-radius: 8px; padding: 14px; margin-bottom: 12px; }';
    html += '.calib-stat-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 12px; margin: 12px 0; }';
    html += '.calib-stat-item { background: white; padding: 10px; border-radius: 6px; text-align: center; border-left: 3px solid #667eea; }';
    html += '.calib-stat-value { font-size: 1.2em; font-weight: bold; margin-top: 4px; }';
    html += '.calib-section-title { font-size: 1.1em; font-weight: 600; color: #333; margin: 16px 0 12px 0; padding-bottom: 8px; border-bottom: 2px solid #667eea; }';
    html += '.calib-code-block { background: #1e1e1e; color: #d4d4d4; padding: 12px; border-radius: 6px; overflow-x: auto; font-family: monospace; font-size: 0.85em; line-height: 1.5; }';
    html += '.calib-highlight-box { padding: 12px; border-radius: 6px; margin: 8px 0; border-left: 3px solid; }';
    html += '.calib-highlight-info { background: #e7f3ff; border-color: #0066cc; }';
    html += '.calib-highlight-warning { background: #fff3cd; border-color: #ffc107; }';
    html += '</style>';
    
    html += '<div style="font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; line-height: 1.5; color: #333;">';
    
    // 基本信息
    html += '<div class="calib-result-card">';
    html += '<h3 style="margin: 0 0 12px 0; font-size: 1.3em; display: flex; align-items: center; gap: 8px;">';
    html += '<span>✅</span> 标定成功';
    html += '</h3>';
    html += '<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; margin-top: 12px;">';
    html += `<div><strong style="opacity: 0.9; font-size: 0.9em;">标定方法：</strong><div style="margin-top: 4px; font-size: 1em;">${result.method || 'Pose-Based (AX=XB)'}</div></div>`;
    html += `<div><strong style="opacity: 0.9; font-size: 0.9em;">标定类型：</strong><div style="margin-top: 4px; font-size: 1em;">${result.calibration_type || 'Eye-in-Hand'}</div></div>`;
    html += `<div><strong style="opacity: 0.9; font-size: 0.9em;">标定时间：</strong><div style="margin-top: 4px; font-size: 1em;">${new Date().toLocaleString('zh-CN')}</div></div>`;
    html += '</div></div>';
    
    // 误差统计
    if (result.evaluation) {
        html += '<div class="calib-info-card">';
        html += '<div class="calib-section-title">📊 标定误差统计</div>';
        
        const meanError = result.evaluation.mean_error;
        const errorColor = meanError < 1.0 ? '#28a745' : meanError < 5.0 ? '#ffc107' : '#dc3545';
        
        html += '<div class="calib-stat-grid">';
        html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.8;">平均误差</div><div class="calib-stat-value" style="color: ${errorColor};">${meanError.toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.8;">最大误差</div><div class="calib-stat-value">${result.evaluation.max_error.toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.8;">最小误差</div><div class="calib-stat-value" style="color: #28a745;">${result.evaluation.min_error.toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.8;">标准差</div><div class="calib-stat-value">${result.evaluation.std_error.toFixed(3)} mm</div></div>`;
        html += '</div>';
        html += '</div>';
    }
    
    // 旋转矩阵和平移向量
    if (result.rotation_matrix && result.translation_vector) {
        // 旋转矩阵
        html += '<div class="calib-info-card">';
        html += '<div class="calib-section-title">📐 旋转矩阵 (3×3)</div>';
        html += '<div class="calib-code-block">';
        result.rotation_matrix.forEach((row, i) => {
            html += `[${row.map(v => v.toFixed(6).padStart(10)).join(', ')}]`;
            if (i < result.rotation_matrix.length - 1) html += '\n';
        });
        html += '</div>';
        
        // 欧拉角
        try {
            const euler = rotationMatrixToEulerAngles(result.rotation_matrix);
            html += '<div class="calib-highlight-box calib-highlight-info" style="margin-top: 12px;">';
            html += '<div style="font-weight: 600; margin-bottom: 8px; color: #0066cc; font-size: 0.95em;">🎯 欧拉角 (ZYX顺序，单位：度)</div>';
            html += '<div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-top: 8px;">';
            html += `<div style="text-align: center; padding: 8px; background: white; border-radius: 4px;"><div style="font-size: 0.8em; opacity: 0.7;">Roll (X)</div><div style="font-size: 1.1em; font-weight: bold; color: #0066cc;">${euler.roll.toFixed(3)}°</div></div>`;
            html += `<div style="text-align: center; padding: 8px; background: white; border-radius: 4px;"><div style="font-size: 0.8em; opacity: 0.7;">Pitch (Y)</div><div style="font-size: 1.1em; font-weight: bold; color: #0066cc;">${euler.pitch.toFixed(3)}°</div></div>`;
            html += `<div style="text-align: center; padding: 8px; background: white; border-radius: 4px;"><div style="font-size: 0.8em; opacity: 0.7;">Yaw (Z)</div><div style="font-size: 1.1em; font-weight: bold; color: #0066cc;">${euler.yaw.toFixed(3)}°</div></div>`;
            html += '</div></div>';
        } catch (e) {
            console.warn('计算欧拉角失败:', e);
        }
        html += '</div>';
        
        // 平移向量
        const translation = result.translation_vector;
        const translationMag = vectorMagnitude(translation);
        html += '<div class="calib-info-card">';
        html += '<div class="calib-section-title">📍 平移向量 (mm)</div>';
        html += '<div class="calib-stat-grid">';
        html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.8;">X分量</div><div class="calib-stat-value">${translation[0].toFixed(3)}</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.8;">Y分量</div><div class="calib-stat-value">${translation[1].toFixed(3)}</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.8;">Z分量</div><div class="calib-stat-value">${translation[2].toFixed(3)}</div></div>`;
        html += `<div class="calib-stat-item" style="border-left-color: #0066cc;"><div style="font-size: 0.85em; opacity: 0.8;">模长</div><div class="calib-stat-value" style="color: #0066cc;">${translationMag.toFixed(3)}</div></div>`;
        html += '</div>';
        html += '<div class="calib-highlight-box calib-highlight-info" style="margin-top: 12px;">';
        html += `<div style="margin-bottom: 6px; font-size: 0.9em;"><strong>向量值：</strong><code style="background: white; padding: 4px 8px; border-radius: 4px; font-size: 0.9em;">[${translation.map(v => v.toFixed(3)).join(', ')}]</code></div>`;
        html += '</div></div>';
    }
    
    html += '</div>';
    
    displayDiv.innerHTML = html;
}

// 显示标定结果确认模态框
// 功能：显示标定结果的详细信息，包括变换矩阵、误差统计等，并提供保存选项
function showCalibrationResultModal() {
    console.log('showCalibrationResultModal called');
    const modal = document.getElementById('calibration-result-modal');
    const content = document.getElementById('calibration-result-content');
    
    console.log('Modal element:', modal);
    console.log('Content element:', content);
    
    if (!modal || !content) {
        addLog('error', '❌ 无法显示标定结果：模态框元素不存在');
        console.error('Modal or content element not found');
        return;
    }
    
    const result = autoCalibState.calibrationResult;
    console.log('Calibration result:', result);
    
    if (!result || !result.success) {
        addLog('error', '❌ 没有可用的标定结果，请先执行标定');
        console.error('No valid calibration result');
        return;
    }
    
    // 构建结果显示HTML - 使用现代化的卡片式设计（包含CSS样式）
    let html = '<style>';
    html += '.calib-result-card { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 20px; border-radius: 12px; margin-bottom: 20px; box-shadow: 0 8px 16px rgba(0,0,0,0.1); }';
    html += '.calib-info-card { background: white; border: 1px solid #e0e0e0; border-radius: 10px; padding: 18px; margin-bottom: 18px; box-shadow: 0 2px 8px rgba(0,0,0,0.05); transition: transform 0.2s, box-shadow 0.2s; }';
    html += '.calib-info-card:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(0,0,0,0.1); }';
    html += '.calib-stat-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; margin: 15px 0; }';
    html += '.calib-stat-item { background: linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%); padding: 12px; border-radius: 8px; text-align: center; border-left: 4px solid #667eea; }';
    html += '.calib-stat-value { font-size: 1.4em; font-weight: bold; margin-top: 5px; }';
    html += '.calib-table { width: 100%; border-collapse: separate; border-spacing: 0; border-radius: 8px; overflow: hidden; box-shadow: 0 2px 8px rgba(0,0,0,0.05); }';
    html += '.calib-table thead { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; }';
    html += '.calib-table th { padding: 10px; text-align: left; font-weight: 600; font-size: 0.9em; }';
    html += '.calib-table td { padding: 8px 10px; border-bottom: 1px solid #f0f0f0; font-size: 0.9em; }';
    html += '.calib-table tbody tr:hover { background: #f8f9fa; }';
    html += '.calib-table tbody tr:last-child td { border-bottom: none; }';
    html += '.calib-badge { display: inline-block; padding: 4px 10px; border-radius: 12px; font-size: 0.8em; font-weight: 600; }';
    html += '.calib-badge-success { background: #d4edda; color: #155724; }';
    html += '.calib-badge-warning { background: #fff3cd; color: #856404; }';
    html += '.calib-badge-danger { background: #f8d7da; color: #721c24; }';
    html += '.calib-code-block { background: #1e1e1e; color: #d4d4d4; padding: 12px; border-radius: 8px; overflow-x: auto; font-family: "Consolas", "Monaco", monospace; font-size: 0.85em; line-height: 1.5; }';
    html += '.calib-details { margin: 10px 0; }';
    html += '.calib-details summary { cursor: pointer; padding: 10px; background: #f8f9fa; border-radius: 6px; font-weight: 600; color: #495057; transition: background 0.2s; font-size: 0.95em; }';
    html += '.calib-details summary:hover { background: #e9ecef; }';
    html += '.calib-details-content { padding: 12px; background: #fcfcfc; border-radius: 0 0 6px 6px; margin-top: 5px; }';
    html += '.calib-highlight-box { padding: 12px; border-radius: 8px; margin: 10px 0; border-left: 4px solid; }';
    html += '.calib-highlight-info { background: #e7f3ff; border-color: #0066cc; }';
    html += '.calib-highlight-warning { background: #fff3cd; border-color: #ffc107; }';
    html += '.calib-section-title { font-size: 1.15em; font-weight: 700; color: #333; margin: 18px 0 12px 0; padding-bottom: 8px; border-bottom: 2px solid #667eea; }';
    html += '</style>';
    
    html += '<div style="font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif; line-height: 1.6; color: #333;">';
    
    // 基本信息 - 使用渐变卡片（简化版）
    html += '<div class="calib-result-card">';
    html += '<h3 style="margin: 0 0 12px 0; font-size: 1.4em; display: flex; align-items: center; gap: 10px;">';
    html += '<span style="font-size: 1.2em;">✅</span> 标定成功';
    html += '</h3>';
    html += '<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(180px, 1fr)); gap: 12px; margin-top: 12px;">';
    html += `<div><strong style="opacity: 0.85; font-size: 0.9em;">标定方法</strong><div style="margin-top: 3px; font-size: 1.05em;">${result.method || 'Pose-Based'}</div></div>`;
    html += `<div><strong style="opacity: 0.85; font-size: 0.9em;">标定类型</strong><div style="margin-top: 3px; font-size: 1.05em;">${result.calibration_type || 'Eye-in-Hand'}</div></div>`;
    html += `<div><strong style="opacity: 0.85; font-size: 0.9em;">标定时间</strong><div style="margin-top: 3px; font-size: 1.05em;">${new Date().toLocaleString('zh-CN', {month: '2-digit', day: '2-digit', hour: '2-digit', minute: '2-digit'})}</div></div>`;
    html += '</div>';
    html += '</div>';
    
    // 误差统计（优化版 - 两种误差类型）
    if (result.evaluation) {
        // 1. AX=XB约束误差（相机坐标系下的标定板姿态变化误差）
        html += '<div class="calib-info-card">';
        html += '<div class="calib-section-title">📊 误差评估（AX=XB约束验证）</div>';
        html += '<div style="font-size: 0.85em; color: #6c757d; margin-bottom: 12px; font-style: italic;">💡 通过相邻姿态对验证标定板在相机坐标系下的姿态变化一致性</div>';
        
        const meanError = result.evaluation.mean_error;
        const errorColor = meanError < 2.0 ? '#28a745' : meanError < 5.0 ? '#ffc107' : '#dc3545';
        const errorBadgeClass = meanError < 2.0 ? 'calib-badge-success' : meanError < 5.0 ? 'calib-badge-warning' : 'calib-badge-danger';
        const errorLevel = meanError < 2.0 ? '优秀' : meanError < 5.0 ? '良好' : '需改进';
        
        html += '<div class="calib-stat-grid">';
        html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.75;">平移RMS误差</div><div class="calib-stat-value" style="color: ${errorColor};">${meanError.toFixed(3)} mm</div><div style="margin-top: 5px;"><span class="calib-badge ${errorBadgeClass}">${errorLevel}</span></div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.75;">最大平移误差</div><div class="calib-stat-value">${result.evaluation.max_error.toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.75;">标准差</div><div class="calib-stat-value">${result.evaluation.std_error.toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.75;">姿态对数量</div><div class="calib-stat-value">${result.evaluation.data_count || autoCalibState.collectedCalibrationData?.length || 'N/A'}</div></div>`;
        html += '</div>';
        
        // 显示各姿态对的误差
        const errorsPerPosePair = result.evaluation.errors_per_pose_pair || [];
        if (errorsPerPosePair.length > 0) {
            html += '<div style="margin-top: 15px; padding: 12px; background: #f8f9fa; border-radius: 8px;">';
            html += '<div style="font-weight: 600; margin-bottom: 8px; color: #495057; font-size: 0.95em;">📋 各姿态对误差</div>';
            html += '<div style="max-height: 200px; overflow-y: auto;">';
            html += '<table class="calib-table" style="width: 100%; font-size: 0.9em;">';
            html += '<thead><tr><th>姿态对</th><th>平移误差 (mm)</th></tr></thead><tbody>';
            errorsPerPosePair.forEach((err, idx) => {
                const errColor = err < 2.0 ? '#28a745' : err < 5.0 ? '#ffc107' : '#dc3545';
                html += `<tr><td>#${idx+1}-${idx+2}</td><td style="color: ${errColor}; font-weight: 600;">${err.toFixed(3)}</td></tr>`;
            });
            html += '</tbody></table>';
            html += '</div></div>';
        }
        
        // 显示旋转误差（如果可用）
        if (result.evaluation.mean_rotation_error_deg !== undefined) {
            const meanRotError = result.evaluation.mean_rotation_error_deg;
            const rotErrorColor = meanRotError < 0.5 ? '#28a745' : meanRotError < 1.0 ? '#ffc107' : '#dc3545';
            const rotErrorBadgeClass = meanRotError < 0.5 ? 'calib-badge-success' : meanRotError < 1.0 ? 'calib-badge-warning' : 'calib-badge-danger';
            const rotErrorLevel = meanRotError < 0.5 ? '优秀' : meanRotError < 1.0 ? '良好' : '需改进';
            
            html += '<div style="margin-top: 15px; padding: 12px; background: #f8f9fa; border-radius: 8px; border-left: 4px solid #667eea;">';
            html += '<div style="font-weight: 600; margin-bottom: 8px; color: #495057; font-size: 0.95em;">🔄 旋转误差（参考 moveit_calibration）</div>';
            html += '<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px;">';
            html += `<div style="text-align: center;"><div style="font-size: 0.8em; opacity: 0.7;">RMS误差</div><div style="font-size: 1.3em; font-weight: bold; color: ${rotErrorColor}; margin-top: 3px;">${meanRotError.toFixed(3)}°</div><span class="calib-badge ${rotErrorBadgeClass}" style="margin-top: 4px; display: inline-block;">${rotErrorLevel}</span></div>`;
            if (result.evaluation.max_rotation_error_deg !== undefined) {
                html += `<div style="text-align: center;"><div style="font-size: 0.8em; opacity: 0.7;">最大误差</div><div style="font-size: 1.3em; font-weight: bold; margin-top: 3px;">${result.evaluation.max_rotation_error_deg.toFixed(3)}°</div></div>`;
            }
            if (result.evaluation.min_rotation_error_deg !== undefined) {
                html += `<div style="text-align: center;"><div style="font-size: 0.8em; opacity: 0.7;">最小误差</div><div style="font-size: 1.3em; font-weight: bold; margin-top: 3px;">${result.evaluation.min_rotation_error_deg.toFixed(3)}°</div></div>`;
            }
            html += '</div>';
            html += '</div>';
        }
        
        html += '</div>';  // 关闭AX=XB约束误差卡片
        
        // 2. 重投影误差（使用标定数据计算）
        const reprojectionErrors = [];
        if (autoCalibState.collectedCalibrationData && autoCalibState.collectedCalibrationData.length > 0) {
            autoCalibState.collectedCalibrationData.forEach(data => {
                if (data.board_pose?.reprojection_error !== undefined) {
                    reprojectionErrors.push(data.board_pose.reprojection_error);
                }
            });
        }
        
        if (reprojectionErrors.length > 0) {
            const meanReproj = reprojectionErrors.reduce((a, b) => a + b, 0) / reprojectionErrors.length;
            const maxReproj = Math.max(...reprojectionErrors);
            const minReproj = Math.min(...reprojectionErrors);
            const stdReproj = Math.sqrt(reprojectionErrors.reduce((sum, err) => sum + Math.pow(err - meanReproj, 2), 0) / reprojectionErrors.length);
            const reprojColor = meanReproj < 1.0 ? '#28a745' : meanReproj < 2.0 ? '#ffc107' : '#dc3545';
            const reprojBadgeClass = meanReproj < 1.0 ? 'calib-badge-success' : meanReproj < 2.0 ? 'calib-badge-warning' : 'calib-badge-danger';
            const reprojLevel = meanReproj < 1.0 ? '优秀' : meanReproj < 2.0 ? '良好' : '需改进';
            
            html += '<div class="calib-info-card" style="margin-top: 15px;">';
            html += '<div class="calib-section-title">🎯 重投影误差（使用标定数据计算）</div>';
            html += '<div style="font-size: 0.85em; color: #6c757d; margin-bottom: 12px; font-style: italic;">💡 使用标定结果将标定板角点投影到图像平面，计算投影点与实际检测点的误差</div>';
            html += '<div class="calib-stat-grid">';
            html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.75;">平均重投影误差</div><div class="calib-stat-value" style="color: ${reprojColor};">${meanReproj.toFixed(3)} px</div><div style="margin-top: 5px;"><span class="calib-badge ${reprojBadgeClass}">${reprojLevel}</span></div></div>`;
            html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.75;">最大重投影误差</div><div class="calib-stat-value">${maxReproj.toFixed(3)} px</div></div>`;
            html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.75;">最小重投影误差</div><div class="calib-stat-value" style="color: #28a745;">${minReproj.toFixed(3)} px</div></div>`;
            html += `<div class="calib-stat-item"><div style="font-size: 0.85em; opacity: 0.75;">标准差</div><div class="calib-stat-value">${stdReproj.toFixed(3)} px</div></div>`;
            html += '</div>';
            html += '<div style="font-size: 0.8em; color: #6c757d; margin-top: 8px; font-style: italic;">💡 重投影误差 < 1 像素为优秀，反映标定板位姿计算质量</div>';
            html += '</div>';
        }
        
        // 显示旋转误差详情（如果可用）
        // 兼容新旧数据结构：优先使用errors_per_pose，如果不存在则使用errors_per_motion
        const errorsPerPose = result.evaluation.errors_per_pose || result.evaluation.errors_per_motion || [];
        const rotationErrorsPerPose = result.evaluation.rotation_errors_per_pose_deg || result.evaluation.rotation_errors_per_motion_deg || [];
        
        if (rotationErrorsPerPose && rotationErrorsPerPose.length > 0) {
            html += '<div style="margin-top: 15px;">';
            html += '<div class="calib-section-title">📋 各姿态误差详情（标定板在基座坐标系中的位姿一致性）</div>';
            html += '<div style="overflow-x: auto;">';
            html += '<table class="calib-table">';
            html += '<thead><tr><th>姿态</th><th>平移误差 (mm)</th><th>旋转误差 (°)</th></tr></thead>';
            html += '<tbody>';
            for (let i = 0; i < errorsPerPose.length; i++) {
                const tErr = errorsPerPose[i];
                const rErr = rotationErrorsPerPose[i];
                const tErrColor = tErr < 2.0 ? '#28a745' : tErr < 5.0 ? '#ffc107' : '#dc3545';
                const rErrColor = rErr < 0.5 ? '#28a745' : rErr < 1.0 ? '#ffc107' : '#dc3545';
                html += `<tr><td>#${i+1}</td><td style="color: ${tErrColor}; font-weight: 600;">${tErr.toFixed(3)}</td><td style="color: ${rErrColor}; font-weight: 600;">${rErr.toFixed(3)}</td></tr>`;
            }
            html += '</tbody>';
            html += '</table>';
            html += '</div>';
            html += '</div>';
        }
        
        // 各姿态误差详情（如果同时有平移和旋转误差，显示完整表格）
        if (errorsPerPose && errorsPerPose.length > 0) {
            html += '<details class="calib-details" style="margin-top: 12px;"><summary>📋 各组误差详情</summary>';
            html += '<div class="calib-details-content" style="max-height: 280px; overflow-y: auto;">';
            html += '<table class="calib-table">';
            
            // 如果有旋转误差，显示完整表格
            if (rotationErrorsPerPose && rotationErrorsPerPose.length > 0) {
                html += '<thead><tr><th style="width: 15%;">姿态</th><th style="width: 25%;">平移误差 (mm)</th><th style="width: 25%;">旋转误差 (°)</th><th style="width: 35%;">评级</th></tr></thead>';
                html += '<tbody>';
                errorsPerPose.forEach((error, idx) => {
                    const rotError = rotationErrorsPerPose[idx] || 0;
                    const errorColor = error < 2.0 ? '#28a745' : error < 5.0 ? '#ffc107' : '#dc3545';
                    const rotErrorColor = rotError < 0.5 ? '#28a745' : rotError < 1.0 ? '#ffc107' : '#dc3545';
                    const badgeClass = (error < 2.0 && rotError < 0.5) ? 'calib-badge-success' : 
                                      (error < 5.0 && rotError < 1.0) ? 'calib-badge-warning' : 'calib-badge-danger';
                    const status = (error < 2.0 && rotError < 0.5) ? '优秀' : 
                                  (error < 5.0 && rotError < 1.0) ? '良好' : '需改进';
                    html += `<tr><td><strong>#${idx + 1}</strong></td>`;
                    html += `<td style="color: ${errorColor}; font-weight: 600;">${error.toFixed(3)}</td>`;
                    html += `<td style="color: ${rotErrorColor}; font-weight: 600;">${rotError.toFixed(3)}</td>`;
                    html += `<td><span class="calib-badge ${badgeClass}">${status}</span></td></tr>`;
                });
            } else {
                // 只有平移误差
                html += '<thead><tr><th style="width: 25%;">姿态</th><th style="width: 35%;">误差 (mm)</th><th style="width: 40%;">评级</th></tr></thead>';
                html += '<tbody>';
                errorsPerPose.forEach((error, idx) => {
                    const errorColor = error < 2.0 ? '#28a745' : error < 5.0 ? '#ffc107' : '#dc3545';
                    const badgeClass = error < 2.0 ? 'calib-badge-success' : error < 5.0 ? 'calib-badge-warning' : 'calib-badge-danger';
                    const status = error < 2.0 ? '优秀' : error < 5.0 ? '良好' : '需改进';
                    html += `<tr><td><strong>#${idx + 1}</strong></td>`;
                    html += `<td style="color: ${errorColor}; font-weight: 600;">${error.toFixed(3)}</td>`;
                    html += `<td><span class="calib-badge ${badgeClass}">${status}</span></td></tr>`;
                });
            }
            html += '</tbody></table></div></details>';
        }
        html += '</div>';
        
        // 运动质量评估（简化版）
        if (result.evaluation.avg_motion_quality !== undefined) {
            const qualityPercent = (result.evaluation.avg_motion_quality * 100).toFixed(1);
            const qualityColor = result.evaluation.avg_motion_quality >= 0.8 ? '#28a745' : 
                               (result.evaluation.avg_motion_quality >= 0.6 ? '#ffc107' : '#dc3545');
            const qualityBadgeClass = result.evaluation.avg_motion_quality >= 0.8 ? 'calib-badge-success' : 
                                    (result.evaluation.avg_motion_quality >= 0.6 ? 'calib-badge-warning' : 'calib-badge-danger');
            
            html += '<div class="calib-info-card">';
            html += '<div class="calib-section-title">⭐ 运动质量</div>';
            html += `<div style="text-align: center; padding: 16px; background: linear-gradient(135deg, ${qualityColor}15 0%, ${qualityColor}05 100%); border-radius: 8px; margin: 12px 0;">`;
            html += `<div style="font-size: 0.85em; opacity: 0.7; margin-bottom: 8px;">平均质量评分</div>`;
            html += `<div style="font-size: 2.2em; font-weight: bold; color: ${qualityColor};">${qualityPercent}%</div>`;
            html += `<span class="calib-badge ${qualityBadgeClass}" style="margin-top: 8px; display: inline-block;">`;
            html += qualityPercent >= 80 ? '优秀' : (qualityPercent >= 60 ? '良好' : '需改进');
            html += '</span></div>';
            
            if (result.evaluation.motion_quality_analysis && result.evaluation.motion_quality_analysis.length > 0) {
                html += '<details class="calib-details"><summary>各组运动详情</summary>';
                html += '<div class="calib-details-content" style="max-height: 280px; overflow-y: auto;">';
                html += '<table class="calib-table">';
                html += '<thead><tr><th style="width: 15%;">组号</th><th style="width: 20%;">质量</th><th style="width: 22%;">平移(mm)</th><th style="width: 22%;">旋转(°)</th><th style="width: 21%;">状态</th></tr></thead>';
                html += '<tbody>';
                result.evaluation.motion_quality_analysis.forEach((motion, idx) => {
                    const motionColor = motion.quality_score >= 0.8 ? '#28a745' : 
                                      (motion.quality_score >= 0.6 ? '#ffc107' : '#dc3545');
                    const motionBadgeClass = motion.quality_score >= 0.8 ? 'calib-badge-success' : 
                                           (motion.quality_score >= 0.6 ? 'calib-badge-warning' : 'calib-badge-danger');
                    const status = motion.quality_score >= 0.8 ? '优秀' : (motion.quality_score >= 0.6 ? '良好' : '差');
                    html += `<tr>`;
                    html += `<td><strong>#${idx + 1}</strong></td>`;
                    html += `<td style="color: ${motionColor}; font-weight: 600;">${(motion.quality_score * 100).toFixed(0)}%</td>`;
                    html += `<td>${motion.translation_mm ? motion.translation_mm.toFixed(1) : 'N/A'}</td>`;
                    html += `<td>${motion.rotation_deg ? motion.rotation_deg.toFixed(1) : 'N/A'}</td>`;
                    html += `<td><span class="calib-badge ${motionBadgeClass}">${status}</span></td>`;
                    html += `</tr>`;
                });
                html += '</tbody></table></div></details>';
            }
            html += '</div>';
        }
    }
    
    // 变换矩阵（简化版）
    if (result.transformation_matrix) {
        html += '<div class="calib-info-card">';
        html += '<div class="calib-section-title">🔢 变换矩阵 (4×4)</div>';
        html += '<details class="calib-details"><summary>查看完整矩阵</summary>';
        html += '<div class="calib-details-content">';
        html += '<div class="calib-code-block">';
        result.transformation_matrix.forEach((row, i) => {
            html += `[${row.map(v => v.toFixed(6).padStart(10)).join(', ')}]`;
            if (i < result.transformation_matrix.length - 1) html += '\n';
        });
        html += '</div>';
        html += '<div style="font-size: 0.8em; color: #6c757d; margin-top: 8px; font-style: italic;">相机坐标系 → 机器人末端坐标系</div>';
        html += '</div></details>';
        html += '</div>';
    }
    
    // 旋转和平移（简化优化版）
    if (result.rotation_matrix && result.translation_vector) {
        html += '<div class="calib-info-card">';
        html += '<div class="calib-section-title">📐 旋转与平移</div>';
        
        // 欧拉角表示（主要显示）
        try {
            const euler = rotationMatrixToEulerAngles(result.rotation_matrix);
            html += '<div style="margin-bottom: 15px;">';
            html += '<div style="font-weight: 600; margin-bottom: 8px; color: #495057; font-size: 0.95em;">🎯 欧拉角 (°)</div>';
            html += '<div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px;">';
            html += `<div style="text-align: center; padding: 10px; background: #f8f9fa; border-radius: 6px;"><div style="font-size: 0.8em; opacity: 0.65; margin-bottom: 4px;">Roll (X)</div><div style="font-size: 1.3em; font-weight: bold; color: #0066cc;">${euler.roll.toFixed(2)}°</div></div>`;
            html += `<div style="text-align: center; padding: 10px; background: #f8f9fa; border-radius: 6px;"><div style="font-size: 0.8em; opacity: 0.65; margin-bottom: 4px;">Pitch (Y)</div><div style="font-size: 1.3em; font-weight: bold; color: #0066cc;">${euler.pitch.toFixed(2)}°</div></div>`;
            html += `<div style="text-align: center; padding: 10px; background: #f8f9fa; border-radius: 6px;"><div style="font-size: 0.8em; opacity: 0.65; margin-bottom: 4px;">Yaw (Z)</div><div style="font-size: 1.3em; font-weight: bold; color: #0066cc;">${euler.yaw.toFixed(2)}°</div></div>`;
            html += '</div></div>';
        } catch (e) {
            console.warn('计算欧拉角失败:', e);
        }
        
        // 平移向量
        const translation = result.translation_vector;
        const translationMag = vectorMagnitude(translation);
        html += '<div>';
        html += '<div style="font-weight: 600; margin-bottom: 8px; color: #495057; font-size: 0.95em;">📍 平移向量 (mm)</div>';
        html += '<div style="display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px;">';
        html += `<div style="text-align: center; padding: 10px; background: #f8f9fa; border-radius: 6px;"><div style="font-size: 0.8em; opacity: 0.65; margin-bottom: 4px;">X</div><div style="font-size: 1.2em; font-weight: bold;">${translation[0].toFixed(2)}</div></div>`;
        html += `<div style="text-align: center; padding: 10px; background: #f8f9fa; border-radius: 6px;"><div style="font-size: 0.8em; opacity: 0.65; margin-bottom: 4px;">Y</div><div style="font-size: 1.2em; font-weight: bold;">${translation[1].toFixed(2)}</div></div>`;
        html += `<div style="text-align: center; padding: 10px; background: #f8f9fa; border-radius: 6px;"><div style="font-size: 0.8em; opacity: 0.65; margin-bottom: 4px;">Z</div><div style="font-size: 1.2em; font-weight: bold;">${translation[2].toFixed(2)}</div></div>`;
        html += `<div style="text-align: center; padding: 10px; background: linear-gradient(135deg, #e7f3ff 0%, #d6eaff 100%); border-radius: 6px; border: 2px solid #0066cc;"><div style="font-size: 0.8em; opacity: 0.7; margin-bottom: 4px;">距离</div><div style="font-size: 1.3em; font-weight: bold; color: #0066cc;">${translationMag.toFixed(1)}</div></div>`;
        html += '</div></div>';
        
        // 详细矩阵（折叠）
        html += '<details class="calib-details" style="margin-top: 12px;"><summary>详细矩阵数据</summary>';
        html += '<div class="calib-details-content">';
        html += '<div style="margin-bottom: 12px;"><strong>旋转矩阵 (3×3)：</strong></div>';
        html += '<div class="calib-code-block" style="margin-bottom: 12px;">';
        result.rotation_matrix.forEach((row, i) => {
            html += `[${row.map(v => v.toFixed(6).padStart(10)).join(', ')}]`;
            if (i < result.rotation_matrix.length - 1) html += '\n';
        });
        html += '</div>';
        
        // 轴角表示
        try {
            const axisAngle = rotationMatrixToAxisAngle(result.rotation_matrix);
            html += '<div style="margin-top: 10px; padding: 8px; background: #fff3cd; border-radius: 6px; font-size: 0.9em;">';
            html += `<strong>轴角：</strong>绕 [${axisAngle.axis.map(v => v.toFixed(3)).join(', ')}] 旋转 ${axisAngle.angle.toFixed(2)}°`;
            html += '</div>';
        } catch (e) {}
        
        html += '</div></details>';
        html += '</div>';
    }
    
    html += '</div>';
    
    content.innerHTML = html;
    modal.style.display = 'block';
    modal.style.zIndex = '10000'; // 确保在最上层
    
    console.log('Modal displayed, z-index:', modal.style.zIndex);
    
    // 绑定确认保存按钮
    const confirmBtn = document.getElementById('btn-confirm-save-calibration');
    if (confirmBtn) {
        confirmBtn.onclick = () => {
            const formatSelect = document.getElementById('calibration-save-format');
            const format = formatSelect ? formatSelect.value : 'yaml';
            closeCalibrationResultModal();
            doSaveCalibration(format);
        };
    } else {
        console.warn('Confirm button not found');
    }
    
    // 添加点击背景关闭功能
    modal.onclick = (e) => {
        if (e.target === modal) {
            closeCalibrationResultModal();
        }
    };
}

// 关闭标定结果模态框
// 功能：隐藏标定结果模态框
function closeCalibrationResultModal() {
    const modal = document.getElementById('calibration-result-modal');
    if (modal) {
        modal.style.display = 'none';
    }
}

// 实际执行保存操作
// 功能：调用后端API保存标定结果到服务器，并下载到本地
// 参数：format-保存格式（yaml或xml）
async function doSaveCalibration(format = 'yaml') {
    addLog('info', `💾 正在保存标定结果（${format.toUpperCase()}格式）...`);
    
    try {
        // 调用后端API生成标定结果并保存到服务器（生成YAML或XML文件）
        const response = await fetch('/api/hand_eye/save_calibration', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
                format: format
            })
        });
        
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        
        const result = await response.json();
        
        // 获取文件内容（支持yaml和xml）
        const fileContent = result.yaml_content || result.xml_content;
        const fileFormat = result.format || format;
        const mimeType = fileFormat === 'yaml' ? 'application/x-yaml' : 'application/xml';
        const fileExtension = fileFormat === 'yaml' ? 'yaml' : 'xml';
        
        if (result.success && fileContent) {
            // 如果成功保存到服务器
            if (result.filename) {
                addLog('success', `💾 标定结果已保存到服务器: ${result.filename}`);
                addLog('info', `   📍 保存路径: config/calibration_results/${result.filename}`);
                showToast(`标定结果已保存: ${result.filename}`, 'success');
            } else {
                // 如果服务器保存失败，回退到本地下载
                addLog('warning', '⚠️ 服务器保存失败，使用本地下载');
            }
            
            // 无论是否保存到服务器，都提供本地下载选项
            const blob = new Blob([fileContent], {type: mimeType});
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = result.filename || `hand_eye_calibration_${Date.now()}.${fileExtension}`;
            a.click();
            URL.revokeObjectURL(url);
            
            if (!result.filename) {
                addLog('success', `✅ 标定结果已下载到本地`);
            }
            addLog('info', `   文件格式: ${fileFormat.toUpperCase()}`);
            addLog('info', '   包含内容: 变换矩阵、标定误差、运动质量分析');
        } else {
            addLog('error', `❌ 保存失败：${result.error || '未知错误'}`);
            showToast(`保存失败: ${result.error || '未知错误'}`, 'error');
        }
        
    } catch (error) {
        addLog('error', `❌ 保存失败：${error.message}`);
        showToast(`保存失败: ${error.message}`, 'error');
        console.error('保存标定结果失败:', error);
    }
}

// 保存标定结果（先显示确认）
// 功能：检查标定结果是否存在，然后显示结果确认模态框（用户可以在模态框中查看结果并保存）
async function handleSaveCalibrationForAuto() {
    console.log('handleSaveCalibrationForAuto called');
    console.log('autoCalibState.calibrationResult:', autoCalibState.calibrationResult);
    
    // 检查是否有标定结果（必须先执行标定）
    if (!autoCalibState.calibrationResult || !autoCalibState.calibrationResult.success) {
        addLog('error', '❌ 没有可用的标定结果，请先执行标定');
        console.error('No calibration result available');
        return;
    }
    
    // 显示标定结果确认模态框（用户查看结果后点击保存）
    try {
        showCalibrationResultModal();
        console.log('Modal should be displayed now');
    } catch (error) {
        console.error('Error showing modal:', error);
        addLog('error', `❌ 显示标定结果失败：${error.message}`);
    }
}

// 注意：旧的UI更新函数已完全移除：
// - updateAutoCalibButtons
// - updateAutoStatus
// - updateAutoProgress
// - updateAutoCurrentStep
// - updateAutoPoseCount
// - updateAutoPoseListTable
// 当前流程使用updateAutoCalibrationProgress和updateRecordedPosesDisplay

// 标定类型切换
// 功能：处理标定类型选择（eye-in-hand或eye-to-hand），更新UI显示
function initAutoCalibTypeChange() {
    const typeSelect = document.getElementById('auto-calibration-type');
    if (typeSelect) {
        typeSelect.addEventListener('change', function() {
            const type = this.value;
            const desc = document.getElementById('auto-calibration-type-desc');
            const heightSetting = document.getElementById('auto-camera-height-setting');
            
            // 根据类型更新描述和高度设置显示
            if (type === 'eye-to-hand') {
                if (desc) desc.textContent = '相机固定，机器人末端点选角点';
                if (heightSetting) heightSetting.style.display = 'block';
            } else {
                if (desc) desc.textContent = '相机安装在机器人末端';
                if (heightSetting) heightSetting.style.display = 'none';
            }
            
            addLog('info', `🔄 切换标定类型：${type === 'eye-to-hand' ? '眼在手外' : '眼在手上'}`);
        });
    }
}

// 复用现有的图像缩放功能
function initImageZoomForTab(tabId) {
    const containerEl = document.getElementById(`${tabId}-image-container`);
    const imageEl = document.getElementById(`${tabId}-image`);
    
    if (!containerEl || !imageEl || !imageZoomStates[tabId]) {
        console.log(`为${tabId}创建新的缩放状态`);
        if (!imageZoomStates[tabId]) {
            imageZoomStates[tabId] = { scale: 1, isDragging: false, lastX: 0, lastY: 0 };
        }
    }
    
    // 触发initImageZoom中的逻辑
    console.log(`${tabId}图像缩放功能已初始化`);
}

// 复用现有的放大镜功能
// 放大镜功能已从自动手眼标定选项卡中移除
// function initMagnifierForTab(tabId) {
//     console.log(`${tabId}放大镜功能已初始化`);
//     // 放大镜功能在initMagnifiers()中已经处理
// }

// 更新智能操作状态卡片
// 功能：根据当前状态（运行中、位姿数量等）更新状态卡片显示和提示信息
function updateAutoStatusCard() {
    const statusCard = document.getElementById('auto-status-card');
    const statusText = document.getElementById('auto-status-text');
    const statusIcon = document.getElementById('auto-status-icon');
    const nextStep = document.getElementById('auto-next-step');
    
    if (!statusCard || !statusText || !statusIcon || !nextStep) return;
    
    // 获取当前状态信息
    const poseCount = autoCalibState.recordedPoses?.length || 0;
    const isRunning = autoCalibState.isRunning || false;
    
    let status = '🎯 准备就绪';
    let icon = '🤖';
    let next = '';
    let bgColor = 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)';
    
    if (isRunning) {
        // 自动标定进行中
        const currentStep = autoCalibState.currentStep || 0;
        const totalSteps = autoCalibState.totalSteps || 0;
        status = `🔄 自动标定进行中`;
        icon = '⏳';
        next = `💡 当前进度: ${currentStep}/${totalSteps} (${Math.round((currentStep/totalSteps)*100)}%)`;
        bgColor = 'linear-gradient(135deg, #4facfe 0%, #00f2fe 100%)';
        
    } else if (poseCount === 0) {
        // 初始状态
        status = '🎯 准备就绪';
        icon = '🤖';
        next = '💡 下一步：移动机器人到第一个位姿，点击"记录机器人位姿"';
        bgColor = 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)';
        
    } else if (poseCount < 3) {
        // 位姿不足
        status = `✅ 已记录 ${poseCount} 个位姿`;
        icon = '📝';
        next = `💡 下一步：继续记录位姿（还需${3-poseCount}个达到最低要求，建议5-8个）`;
        bgColor = 'linear-gradient(135deg, #fa709a 0%, #fee140 100%)';
        
    } else {
        // 位姿足够，可以开始标定
        status = `✅ 已记录 ${poseCount} 个位姿`;
        icon = '✨';
        next = '💡 下一步：点击"开始自动标定"按钮，系统将自动移动到每个位姿并采集数据';
        bgColor = 'linear-gradient(135deg, #30cfd0 0%, #330867 100%)';
    }
    
    statusText.textContent = status;
    statusIcon.textContent = icon;
    nextStep.textContent = next;
    statusCard.style.background = bgColor;
}

// 注意：evaluateMotionQuality函数已移除
// 运动质量评估功能已集成到后端标定计算中


// 辅助函数：延时
// 功能：异步延时函数，用于等待指定时间（毫秒）
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// ============= 模态框关闭事件监听 =============
// 点击模态框外部关闭
window.addEventListener('click', function(event) {
    const modal = document.getElementById('calibration-result-modal');
    if (event.target === modal) {
        closeCalibrationResultModal();
    }
});

// ESC键关闭模态框
window.addEventListener('keydown', function(event) {
    if (event.key === 'Escape') {
        const modal = document.getElementById('calibration-result-modal');
        if (modal && modal.style.display === 'block') {
            closeCalibrationResultModal();
        }
    }
});

// ============= 在现有初始化函数中添加调用 =============
// 需要在 DOMContentLoaded 事件中调用 initAutoHandEyeCalibTab()

console.log('✅ 自动手眼标定扩展模块加载完成');

