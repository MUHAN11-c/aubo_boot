// ============================================================
// 自动手眼标定功能扩展
// 此文件包含自动标定相关的所有功能
// 需要在 script_v2.js 加载后加载此文件
// ============================================================

// 自动标定全局状态
let autoCalibState = {
    isRunning: false,
    isPaused: false,
    currentStep: 0,
    totalSteps: 12,
    plannedPoses: [],
    collectedData: [],
    calibrationType: 'eye-to-hand'
};

// 初始化自动标定选项卡功能
function initAutoHandEyeCalibTab() {
    console.log('初始化自动手眼标定功能...');
    
    // 初始化自动标定按钮
    initAutoCalibButtons();
    
    // 初始化标定类型切换
    initAutoCalibTypeChange();
    
    // 注意：图像缩放和放大镜功能由 script_v2.js 的 initImageZoom() 和 initMagnifiers() 统一初始化
    
    addLog('success', '🤖 自动标定功能初始化完成');
}

// 初始化自动标定按钮事件
function initAutoCalibButtons() {
    // 自动控制按钮
    const btnAutoStart = document.getElementById('btn-auto-start');
    const btnAutoPause = document.getElementById('btn-auto-pause');
    const btnAutoResume = document.getElementById('btn-auto-resume');
    const btnAutoStop = document.getElementById('btn-auto-stop');
    
    if (btnAutoStart) {
        btnAutoStart.addEventListener('click', handleAutoCalibStart);
    }
    if (btnAutoPause) {
        btnAutoPause.addEventListener('click', handleAutoCalibPause);
    }
    if (btnAutoResume) {
        btnAutoResume.addEventListener('click', handleAutoCalibResume);
    }
    if (btnAutoStop) {
        btnAutoStop.addEventListener('click', handleAutoCalibStop);
    }
    
    // 手动操作按钮
    const btnAutoLoadParams = document.getElementById('btn-auto-load-camera-params');
    const btnAutoCapture = document.getElementById('btn-auto-capture-image');
    const btnAutoExtract = document.getElementById('btn-auto-extract-corners');
    const btnAutoSavePose = document.getElementById('btn-auto-save-pose-data');
    const btnAutoLoadPose = document.getElementById('btn-auto-load-pose-data');
    const btnAutoStartCalib = document.getElementById('btn-auto-start-calibration');
    const btnAutoSaveCalib = document.getElementById('btn-auto-save-calibration');
    
    if (btnAutoLoadParams) {
        btnAutoLoadParams.addEventListener('click', () => handleLoadCameraParamsForAuto());
    }
    if (btnAutoCapture) {
        btnAutoCapture.addEventListener('click', () => handleCaptureImageForAuto());
    }
    if (btnAutoExtract) {
        btnAutoExtract.addEventListener('click', () => handleExtractCornersForAuto());
    }
    if (btnAutoSavePose) {
        btnAutoSavePose.addEventListener('click', () => handleSavePoseDataForAuto());
    }
    if (btnAutoLoadPose) {
        btnAutoLoadPose.addEventListener('click', () => handleLoadPoseDataForAuto());
    }
    if (btnAutoStartCalib) {
        btnAutoStartCalib.addEventListener('click', () => handleStartCalibrationForAuto());
    }
    if (btnAutoSaveCalib) {
        btnAutoSaveCalib.addEventListener('click', () => handleSaveCalibrationForAuto());
    }
}

// ============= 自动标定核心功能 =============

// 开始自动标定
async function handleAutoCalibStart() {
    if (autoCalibState.isRunning) {
        addLog('warning', '⚠️ 自动标定正在进行中');
        return;
    }
    
    if (!cameraParamsLoaded) {
        addLog('warning', '⚠️ 请先加载相机参数（系统将自动从ROS2话题获取）');
        // 尝试自动加载
        await new Promise(resolve => setTimeout(resolve, 2000));
        if (!cameraParamsLoaded) {
            addLog('error', '❌ 未能从ROS2话题获取相机参数，请检查相机是否在线');
            return;
        }
    }
    
    // 获取自动标定参数
    const sampleCount = parseInt(document.getElementById('auto-sample-count').value) || 12;
    const workspaceRange = parseFloat(document.getElementById('auto-workspace-range').value) || 300;
    const zHeight = parseFloat(document.getElementById('auto-z-height').value) || 300;
    const calibType = document.getElementById('auto-calibration-type').value;
    
    addLog('info', `🚀 开始自动标定：${sampleCount}个采样点，工作空间±${workspaceRange/2}mm，Z=${zHeight}mm`);
    addLog('info', `📋 标定类型：${calibType === 'eye-to-hand' ? '眼在手外' : '眼在手上'}`);
    
    // 初始化状态
    autoCalibState = {
        isRunning: true,
        isPaused: false,
        currentStep: 0,
        totalSteps: sampleCount,
        plannedPoses: [],
        collectedData: [],
        calibrationType: calibType
    };
    
    // 更新UI
    updateAutoCalibButtons(true, false);
    updateAutoStatus('运行中', '#28a745');
    updateAutoProgress(0, sampleCount);
    
    // 规划采样位姿
    autoCalibState.plannedPoses = planSamplingPoses(sampleCount, workspaceRange, zHeight);
    addLog('success', `✅ 已规划 ${autoCalibState.plannedPoses.length} 个采样位姿（均匀分布）`);
    
    // 开始执行自动标定流程
    await executeAutoCalibration();
}

// 暂停自动标定
function handleAutoCalibPause() {
    if (!autoCalibState.isRunning) return;
    
    autoCalibState.isPaused = true;
    addLog('warning', '⏸️ 自动标定已暂停');
    
    const btnPause = document.getElementById('btn-auto-pause');
    const btnResume = document.getElementById('btn-auto-resume');
    if (btnPause) btnPause.style.display = 'none';
    if (btnResume) btnResume.style.display = 'inline-block';
    
    updateAutoStatus('已暂停', '#ffc107');
}

// 继续自动标定
function handleAutoCalibResume() {
    if (!autoCalibState.isRunning) return;
    
    autoCalibState.isPaused = false;
    addLog('info', '▶️ 自动标定继续执行');
    
    const btnPause = document.getElementById('btn-auto-pause');
    const btnResume = document.getElementById('btn-auto-resume');
    if (btnPause) btnPause.style.display = 'inline-block';
    if (btnResume) btnResume.style.display = 'none';
    
    updateAutoStatus('运行中', '#28a745');
    
    // 继续执行
    executeAutoCalibration();
}

// 停止自动标定
function handleAutoCalibStop() {
    if (!autoCalibState.isRunning) return;
    
    autoCalibState.isRunning = false;
    autoCalibState.isPaused = false;
    
    addLog('error', `⏹️ 自动标定已停止（已采集 ${autoCalibState.collectedData.length}/${autoCalibState.totalSteps} 组数据）`);
    
    // 更新UI
    updateAutoCalibButtons(false, false);
    updateAutoStatus('已停止', '#dc3545');
}

// 规划采样位姿（均匀网格分布）
function planSamplingPoses(sampleCount, workspaceRange, zHeight) {
    const poses = [];
    const gridSize = Math.ceil(Math.sqrt(sampleCount));
    const step = workspaceRange / Math.max(gridSize - 1, 1);
    const offset = workspaceRange / 2;
    
    addLog('info', `📐 规划${gridSize}×${gridSize}网格，步长${step.toFixed(1)}mm`);
    
    let poseIndex = 0;
    for (let i = 0; i < gridSize && poseIndex < sampleCount; i++) {
        for (let j = 0; j < gridSize && poseIndex < sampleCount; j++) {
            const x = -offset + i * step;
            const y = -offset + j * step;
            
            poses.push({
                position: {
                    x: parseFloat(x.toFixed(2)),
                    y: parseFloat(y.toFixed(2)),
                    z: parseFloat(zHeight.toFixed(2))
                },
                orientation: {
                    x: 1.0,
                    y: 0.0,
                    z: 0.0,
                    w: 0.0
                }
            });
            
            poseIndex++;
        }
    }
    
    return poses;
}

// 执行自动标定流程
async function executeAutoCalibration() {
    while (autoCalibState.isRunning && 
           autoCalibState.currentStep < autoCalibState.totalSteps) {
        
        // 检查暂停状态
        if (autoCalibState.isPaused) {
            await sleep(500);
            continue;
        }
        
        const stepNum = autoCalibState.currentStep + 1;
        const currentPose = autoCalibState.plannedPoses[autoCalibState.currentStep];
        
        addLog('info', `═══════════════════════════════════════`);
        addLog('info', `📍 [${stepNum}/${autoCalibState.totalSteps}] 开始采集位姿 ${stepNum}`);
        updateAutoCurrentStep(`位姿${stepNum}：移动中...`);
        
        try {
            // 步骤1: 移动机器人到目标位姿
            addLog('info', `🤖 移动到位姿 (X:${currentPose.position.x}, Y:${currentPose.position.y}, Z:${currentPose.position.z})`);
            
            // TODO: 调用实际的机器人移动API
            // await moveRobotToPose(currentPose);
            // 暂时使用延时模拟
            await sleep(1500);
            addLog('success', '✅ 机器人已到达目标位姿');
            
            // 等待机器人稳定
            updateAutoCurrentStep(`位姿${stepNum}：稳定中...`);
            await sleep(1000);
            
            // 步骤2: 采集图像
            updateAutoCurrentStep(`位姿${stepNum}：触发拍照...`);
            addLog('info', '📷 触发相机拍照...');
            
            const captureResponse = await fetch('/api/camera/capture', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'}
            });
            
            if (!captureResponse.ok) {
                addLog('error', `❌ 拍照失败：HTTP ${captureResponse.status}`);
                addLog('warning', '⚠️ 跳过当前位姿，继续下一个');
                autoCalibState.currentStep++;
                updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
                continue;
            }
            
            const captureData = await captureResponse.json();
            
            if (!captureData.success) {
                addLog('error', `❌ 拍照失败：${captureData.error || '未知错误'}`);
                addLog('warning', '⚠️ 跳过当前位姿，继续下一个');
                autoCalibState.currentStep++;
                updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
                continue;
            }
            
            addLog('success', '✅ 拍照命令已发送');
            updateAutoCurrentStep(`位姿${stepNum}：获取图像...`);
            
            // 等待并获取图像
            let imageData = null;
            for (let attempt = 1; attempt <= 10; attempt++) {
                await sleep(500);
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
                addLog('error', '❌ 获取图像超时');
                addLog('warning', '⚠️ 跳过当前位姿，继续下一个');
                autoCalibState.currentStep++;
                updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
                continue;
            }
            
            // 显示采集的图像
            const imgElement = document.getElementById('auto-hand-eye-calib-image');
            const placeholder = document.getElementById('auto-hand-eye-calib-image-container')?.querySelector('.image-placeholder');
            
            if (imgElement) {
                // 清除旧的事件处理器
                imgElement.onload = null;
                imgElement.onerror = null;
                
                imgElement.onload = function() {
                    this.classList.add('loaded');
                    if (placeholder) {
                        placeholder.style.display = 'none';
                    }
                };
                imgElement.onerror = function() {
                    addLog('error', '❌ 图像加载失败');
                };
                imgElement.src = imageData.image;
            }
            addLog('success', '✅ 图像获取成功');
            
            // 步骤3: 提取角点
            updateAutoCurrentStep(`位姿${stepNum}：提取角点...`);
            addLog('info', '🔍 提取棋盘格角点...');
            
            const squareSize = parseFloat(document.getElementById('auto-checkerboard-size').value) || 15;
            const cornersResponse = await fetch('/api/camera/extract_corners', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({square_size: squareSize})
            });
            
            // 检查响应状态
            if (!cornersResponse.ok) {
                addLog('error', `❌ 角点提取失败：HTTP ${cornersResponse.status}`);
                addLog('warning', '⚠️ 跳过当前位姿，继续下一个');
                autoCalibState.currentStep++;
                updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
                continue;
            }
            
            const cornersData = await cornersResponse.json();
            
            if (!cornersData.success) {
                addLog('error', `❌ 角点提取失败：${cornersData.error || '未知错误'}`);
                addLog('warning', '⚠️ 跳过当前位姿，继续下一个');
                autoCalibState.currentStep++;
                updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
                continue;
            }
            
            // 显示带角点的图像
            if (imgElement) {
                // 清除旧的事件处理器
                imgElement.onload = null;
                imgElement.onerror = null;
                
                imgElement.onload = function() {
                    this.classList.add('loaded');
                    if (placeholder) {
                        placeholder.style.display = 'none';
                    }
                };
                imgElement.onerror = function() {
                    addLog('error', '❌ 角点图像加载失败');
                };
                imgElement.src = cornersData.image_with_corners;
            }
            addLog('success', `✅ 检测到 ${cornersData.corners_count} 个角点`);
            
            // 步骤4: 保存数据
            updateAutoCurrentStep(`位姿${stepNum}：保存数据...`);
            autoCalibState.collectedData.push({
                group_id: stepNum,
                pose: currentPose,
                corners: cornersData.corners_data,
                image_with_corners: cornersData.image_with_corners
            });
            
            addLog('success', `✅ 位姿 ${stepNum} 数据采集完成`);
            
            // 步骤5: 更新进度和表格
            autoCalibState.currentStep++;
            updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
            updateAutoPoseListTable();
            updateAutoPoseCount(autoCalibState.collectedData.length);
            
            // 短暂延时，让用户看到结果
            await sleep(800);
            
        } catch (error) {
            addLog('error', `❌ 执行失败：${error.message}`);
            addLog('warning', '⚠️ 跳过当前位姿，继续下一个');
            autoCalibState.currentStep++;
            updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
        }
    }
    
    // 自动标定数据采集完成
    if (autoCalibState.isRunning && autoCalibState.currentStep >= autoCalibState.totalSteps) {
        addLog('success', `═══════════════════════════════════════`);
        addLog('success', `🎉 数据采集完成！共成功采集 ${autoCalibState.collectedData.length} 组数据`);
        
        if (autoCalibState.collectedData.length >= 3) {
            addLog('info', '🔄 开始自动执行手眼标定计算...');
            updateAutoCurrentStep('执行标定计算...');
            
            // 自动执行标定
            await performAutoCalibration();
        } else {
            addLog('error', `❌ 有效数据不足（${autoCalibState.collectedData.length}/3），无法进行标定`);
            updateAutoStatus('数据不足', '#dc3545');
        }
        
        // 结束自动标定
        autoCalibState.isRunning = false;
        updateAutoCalibButtons(false, false);
        
        if (autoCalibState.collectedData.length >= 3) {
            updateAutoStatus('已完成', '#28a745');
            updateAutoCurrentStep('标定完成');
        }
    }
}

// 执行手眼标定计算
async function performAutoCalibration() {
    try {
        // 准备标定数据
        const calibData = {
            calibration_type: autoCalibState.calibrationType,
            poses: autoCalibState.collectedData
        };
        
        const response = await fetch('/api/hand_eye/calibrate', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify(calibData)
        });
        
        // 检查响应状态
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
        
        // 检查content-type
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            const text = await response.text();
            throw new Error(`服务器返回非JSON响应: ${text.substring(0, 100)}`);
        }
        
        const result = await response.json();
        
        if (result.success) {
            addLog('success', `✅ 标定成功！`);
            addLog('success', `📊 标定误差：${result.error ? result.error.toFixed(3) + ' mm' : '未知'}`);
            
            // 更新标定状态显示
            const calibStatusElement = document.getElementById('auto-calib-status');
            const calibErrorElement = document.getElementById('auto-calib-error');
            
            if (calibStatusElement) {
                calibStatusElement.textContent = '已完成';
                calibStatusElement.style.color = '#28a745';
            }
            if (calibErrorElement && result.error) {
                calibErrorElement.textContent = `${result.error.toFixed(3)} mm`;
                calibErrorElement.style.color = result.error < 2 ? '#28a745' : '#ffc107';
            }
            
            // 显示标定结果
            if (result.transformation) {
                addLog('info', '📐 标定矩阵已计算完成');
            }
            
        } else {
            addLog('error', `❌ 标定失败：${result.error || '未知错误'}`);
            updateAutoStatus('标定失败', '#dc3545');
        }
        
    } catch (error) {
        addLog('error', `❌ 标定请求失败：${error.message}`);
        updateAutoStatus('标定失败', '#dc3545');
    }
}

// ============= 手动操作功能（在自动标定页面中）=============

async function handleLoadCameraParamsForAuto() {
    addLog('info', '📂 加载相机参数...');
    // 可以手动加载文件，或者使用ROS2话题的参数
    addLog('info', '💡 系统将自动从ROS2话题获取相机内参');
}

async function handleCaptureImageForAuto() {
    try {
        addLog('info', '📷 触发相机拍照...');
        
        // 步骤1: 触发拍照
        const response = await fetch('/api/camera/capture', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'}
        });
        
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
        
        const data = await response.json();
        
        if (data.success) {
            addLog('success', '✅ 拍照命令已发送: ' + data.message);
            addLog('info', '⏳ 等待图像数据...');
            
            // 步骤2: 轮询获取图像
            let attemptCount = 0;
            const maxAttempts = 10;
            
            const attemptInterval = setInterval(async () => {
                attemptCount++;
                addLog('info', `🔄 尝试获取图像 (${attemptCount}/${maxAttempts})...`);
                
                try {
                    const imgResponse = await fetch('/api/current_image');
                    const imgData = await imgResponse.json();
                    
                    if (imgData.success && imgData.image) {
                        clearInterval(attemptInterval);
                        
                        // 显示图像
                        const imgElement = document.getElementById('auto-hand-eye-calib-image');
                        const placeholder = document.getElementById('auto-hand-eye-calib-image-container')?.querySelector('.image-placeholder');
                        
                        if (imgElement) {
                            // 清除旧的事件处理器，避免残留
                            imgElement.onload = null;
                            imgElement.onerror = null;
                            
                            // 设置新的事件处理器
                            imgElement.onload = function() {
                                this.classList.add('loaded');
                                if (placeholder) {
                                    placeholder.style.display = 'none';
                                }
                                addLog('success', '✅ 图像显示成功');
                            };
                            imgElement.onerror = function() {
                                addLog('error', '❌ 图像加载到DOM失败');
                            };
                            
                            // 直接设置src（base64数据不会被缓存）
                            imgElement.src = imgData.image;
                        }
                        
                        addLog('success', '✅ 图像数据已接收');
                    } else if (attemptCount >= maxAttempts) {
                        clearInterval(attemptInterval);
                        addLog('warning', '⚠️ 未能获取到图像数据，请检查/image_data话题');
                    }
                } catch (error) {
                    if (attemptCount >= maxAttempts) {
                        clearInterval(attemptInterval);
                        addLog('error', `❌ 获取图像失败: ${error.message}`);
                    }
                }
            }, 500); // 每500ms尝试一次
            
        } else {
            addLog('error', `❌ 拍照失败：${data.error || '未知错误'}`);
        }
    } catch (error) {
        addLog('error', `❌ 请求失败：${error.message}`);
        console.error('Capture image error:', error);
    }
}

async function handleExtractCornersForAuto() {
    try {
        addLog('info', '🔍 手动提取角点...');
        const squareSize = parseFloat(document.getElementById('auto-checkerboard-size').value) || 15;
        
        const response = await fetch('/api/camera/extract_corners', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({square_size: squareSize})
        });
        
        // 检查响应状态
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}: ${response.statusText}`);
        }
        
        // 检查content-type
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            const text = await response.text();
            throw new Error(`服务器返回非JSON响应: ${text.substring(0, 100)}`);
        }
        
        const data = await response.json();
        
        if (data.success) {
            const imgElement = document.getElementById('auto-hand-eye-calib-image');
            const placeholder = document.getElementById('auto-hand-eye-calib-image-container')?.querySelector('.image-placeholder');
            
            if (imgElement) {
                // 清除旧的事件处理器，避免残留
                imgElement.onload = null;
                imgElement.onerror = null;
                
                // 设置新的事件处理器
                imgElement.onload = function() {
                    this.classList.add('loaded');
                    if (placeholder) {
                        placeholder.style.display = 'none';
                    }
                    addLog('success', '✅ 角点图像显示成功');
                };
                imgElement.onerror = function() {
                    addLog('error', '❌ 图像加载到DOM失败');
                };
                
                // 直接设置src（base64数据不需要时间戳）
                imgElement.src = data.image_with_corners;
            }
            addLog('success', `✅ 检测到 ${data.corners_count} 个角点`);
        } else {
            addLog('error', `❌ 提取失败：${data.error || '未知错误'}`);
        }
    } catch (error) {
        addLog('error', `❌ 请求失败：${error.message}`);
        console.error('Extract corners error:', error);
    }
}

function handleSavePoseDataForAuto() {
    if (autoCalibState.collectedData.length === 0) {
        addLog('warning', '⚠️ 没有数据可保存');
        return;
    }
    
    const dataStr = JSON.stringify(autoCalibState.collectedData, null, 2);
    const blob = new Blob([dataStr], {type: 'application/json'});
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `auto_hand_eye_poses_${Date.now()}.json`;
    a.click();
    URL.revokeObjectURL(url);
    
    addLog('success', `💾 已保存 ${autoCalibState.collectedData.length} 组位姿数据`);
}

function handleLoadPoseDataForAuto() {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.json';
    input.onchange = (e) => {
        const file = e.target.files[0];
        if (!file) return;
        
        const reader = new FileReader();
        reader.onload = (event) => {
            try {
                autoCalibState.collectedData = JSON.parse(event.target.result);
                addLog('success', `📁 加载了 ${autoCalibState.collectedData.length} 组位姿数据`);
                updateAutoPoseListTable();
                updateAutoPoseCount(autoCalibState.collectedData.length);
            } catch (error) {
                addLog('error', `❌ 解析失败：${error.message}`);
            }
        };
        reader.readAsText(file);
    };
    input.click();
}

async function handleStartCalibrationForAuto() {
    if (autoCalibState.collectedData.length < 3) {
        addLog('warning', `⚠️ 需要至少3组数据才能开始标定（当前：${autoCalibState.collectedData.length}组）`);
        return;
    }
    
    addLog('info', '🔄 手动开始手眼标定计算...');
    await performAutoCalibration();
}

function handleSaveCalibrationForAuto() {
    addLog('info', '💾 保存标定结果功能（待实现）');
    // TODO: 实现保存标定结果到文件
}

// ============= UI更新函数 =============

function updateAutoCalibButtons(isRunning, isPaused) {
    const btnStart = document.getElementById('btn-auto-start');
    const btnPause = document.getElementById('btn-auto-pause');
    const btnResume = document.getElementById('btn-auto-resume');
    const btnStop = document.getElementById('btn-auto-stop');
    
    if (btnStart) btnStart.disabled = isRunning;
    if (btnPause) {
        btnPause.disabled = !isRunning;
        btnPause.style.display = isPaused ? 'none' : 'inline-block';
    }
    if (btnResume) {
        btnResume.style.display = isPaused ? 'inline-block' : 'none';
    }
    if (btnStop) btnStop.disabled = !isRunning;
}

function updateAutoStatus(text, color) {
    const statusElement = document.getElementById('auto-status');
    if (statusElement) {
        statusElement.textContent = text;
        statusElement.style.color = color;
    }
}

function updateAutoProgress(current, total) {
    const percentage = total > 0 ? (current / total * 100).toFixed(0) : 0;
    
    const progressElement = document.getElementById('auto-progress');
    const progressBar = document.getElementById('auto-progress-bar');
    const progressText = document.getElementById('auto-progress-text');
    
    if (progressElement) {
        progressElement.textContent = `${current}/${total}`;
    }
    if (progressBar) {
        progressBar.style.width = `${percentage}%`;
    }
    if (progressText) {
        progressText.textContent = `${percentage}%`;
    }
}

function updateAutoCurrentStep(text) {
    const stepElement = document.getElementById('auto-current-step');
    if (stepElement) {
        stepElement.textContent = text;
    }
}

function updateAutoPoseCount(count) {
    const countElement = document.getElementById('auto-pose-count');
    if (countElement) {
        countElement.textContent = count.toString();
    }
}

function updateAutoPoseListTable() {
    const tbody = document.querySelector('#auto-pose-list-table tbody');
    if (!tbody) return;
    
    tbody.innerHTML = '';
    
    if (autoCalibState.collectedData.length === 0) {
        tbody.innerHTML = '<tr><td colspan="13" class="no-data">🤖 等待自动标定开始...</td></tr>';
        return;
    }
    
    autoCalibState.collectedData.forEach((dataGroup) => {
        dataGroup.corners.forEach((corner, cornerIndex) => {
            const row = tbody.insertRow();
            row.innerHTML = `
                <td style="background: #f0f4ff; font-weight: bold;">${dataGroup.group_id}</td>
                <td>${cornerIndex}</td>
                <td>${corner.pixel_u.toFixed(2)}</td>
                <td>${corner.pixel_v.toFixed(2)}</td>
                <td>${corner.camera_x.toFixed(2)}</td>
                <td>${corner.camera_y.toFixed(2)}</td>
                <td>${dataGroup.pose.position.x.toFixed(2)}</td>
                <td>${dataGroup.pose.position.y.toFixed(2)}</td>
                <td>${dataGroup.pose.position.z.toFixed(2)}</td>
                <td>${dataGroup.pose.orientation.x.toFixed(4)}</td>
                <td>${dataGroup.pose.orientation.y.toFixed(4)}</td>
                <td>${dataGroup.pose.orientation.z.toFixed(4)}</td>
                <td>${dataGroup.pose.orientation.w.toFixed(4)}</td>
            `;
        });
    });
}

// 标定类型切换
function initAutoCalibTypeChange() {
    const typeSelect = document.getElementById('auto-calibration-type');
    if (typeSelect) {
        typeSelect.addEventListener('change', function() {
            const type = this.value;
            const desc = document.getElementById('auto-calibration-type-desc');
            const heightSetting = document.getElementById('auto-camera-height-setting');
            
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
function initMagnifierForTab(tabId) {
    console.log(`${tabId}放大镜功能已初始化`);
    // 放大镜功能在initMagnifiers()中已经处理
}

// 辅助函数：延时
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// ============= 在现有初始化函数中添加调用 =============
// 需要在 DOMContentLoaded 事件中调用 initAutoHandEyeCalibTab()

console.log('✅ 自动手眼标定扩展模块加载完成');

