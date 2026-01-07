// ============================================================
// 自动手眼标定功能扩展
// 此文件包含自动标定相关的所有功能
// 需要在 script_v2.js 加载后加载此文件
// ============================================================

// 自动标定全局状态（完全自动化模式）
let autoCalibState = {
    isRunning: false,
    isPaused: false,
    currentStep: 0,
    totalSteps: 0,              // 动态设置：等于recordedPoses的数量
    calibrationType: 'eye-in-hand',
    calibrationMethod: 'pose-based',  // 使用姿态法
    // 自动化数据
    recordedPoses: [],          // 记录的机器人位姿列表（用于自动运动）
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
    
    // 注意：图像缩放和放大镜功能由 script_v2.js 的 initImageZoom() 和 initMagnifiers() 统一初始化
    
    addLog('success', '🤖 自动标定功能初始化完成');
}

// 初始化自动标定按钮事件
function initAutoCalibButtons() {
    // 位姿管理按钮
    const btnRecordRobotPose = document.getElementById('btn-auto-record-robot-pose');
    const btnClearMotionData = document.getElementById('btn-auto-clear-motion-data');
    const btnSaveAllPoses = document.getElementById('btn-auto-save-all-poses');
    const btnLoadAllPoses = document.getElementById('btn-auto-load-all-poses');
    
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
    
    // 标定执行按钮
    const btnAutoStartCalib = document.getElementById('btn-auto-start-calibration');
    const btnAutoSaveCalib = document.getElementById('btn-auto-save-calibration');
    
    if (btnAutoStartCalib) {
        btnAutoStartCalib.addEventListener('click', () => {
            handleAutoCalibStart();
        });
    }
    if (btnAutoSaveCalib) {
        btnAutoSaveCalib.addEventListener('click', () => handleSaveCalibrationForAuto());
    }
}

// ============= 自动标定功能 =============

// 记录机器人位姿（先显示图像预览，再记录位姿）
async function handleRecordRobotPose() {
    addLog('info', '📡 正在记录机器人位姿...');
    
    try {
        // 步骤1: 先采集并显示图像预览
        addLog('info', '   📷 采集图像预览...');
        
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
        
        // 等待图像可用并显示（优化：减少轮询间隔和最大尝试次数）
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
                addLog('error', '   ❌ 图像加载失败');
            };
            
            // 设置图像源（base64数据不需要时间戳）
            imgElement.src = imageData.image;
        }
        
        // 步骤2: 获取机器人位姿
        addLog('info', '   🤖 获取机器人位姿...');
        
        const robotResponse = await fetch('/api/robot_status');
        
        if (!robotResponse.ok) {
            throw new Error(`HTTP ${robotResponse.status}`);
        }
        
        const robotData = await robotResponse.json();
        
        if (!robotData.success || !robotData.is_online) {
            throw new Error('机器人未在线');
        }
        
        // 保存机器人位姿（深拷贝避免引用共享）
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
        
        addLog('success', `✅ 已记录位姿 #${poseCount}`);
        addLog('info', `   位置: X=${recordedPose.position.x.toFixed(3)}m, Y=${recordedPose.position.y.toFixed(3)}m, Z=${recordedPose.position.z.toFixed(3)}m`);
        addLog('info', `   姿态: X=${recordedPose.orientation.x.toFixed(6)}, Y=${recordedPose.orientation.y.toFixed(6)}, Z=${recordedPose.orientation.z.toFixed(6)}, W=${recordedPose.orientation.w.toFixed(6)}`);
        addLog('info', `💡 当前已记录 ${poseCount} 个位姿（至少需要3个，建议5-8个）`);
        
        // 更新进度条总数
        autoCalibState.totalSteps = poseCount;
        
        // 更新UI显示
        updateRecordedPosesDisplay();
        updateAutoStatusCard();
        
    } catch (error) {
        addLog('error', `❌ 记录位姿失败：${error.message}`);
        console.error('记录位姿失败:', error);
    }
}

// 注意：handleCaptureAndExtractBoard函数已移除
// 标定板位姿提取功能已集成到自动标定流程的captureImageAndExtractCorners函数中

// 清空记录的位姿数据
function handleClearMotionData() {
    const count = autoCalibState.recordedPoses?.length || 0;
    autoCalibState.recordedPoses = [];
    autoCalibState.collectedCalibrationData = [];
    autoCalibState.totalSteps = 0;
    autoCalibState.currentStep = 0;
    
    addLog('info', `🗑️ 已清除 ${count} 个记录的位姿`);
    
    // 更新UI显示
    updateRecordedPosesDisplay();
    updateAutoCalibrationProgress();
    updateAutoStatusCard();
    updateAutoCalibrationDataDisplay(null);
}

// 显示记录的位姿列表
function updateRecordedPosesDisplay() {
    const listDiv = document.getElementById('auto-motion-groups-list');
    const countSpan = document.getElementById('auto-motion-groups-count');
    
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
    } else {
        listDiv.innerHTML = `
            <div style="padding: 10px; background: white; border-radius: 4px; text-align: center; color: #666; font-size: 13px;">
                已记录 ${poseCount} 个位姿
                </div>
            `;
    }
}

// 更新自动标定进度条
function updateAutoCalibrationProgress() {
    const progressDiv = document.getElementById('auto-motion-progress');
    
    if (!progressDiv) return;
    
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

// 保存记录的位姿到文件（v4.0格式）
function handleSaveAllPosesToFile() {
    if (!autoCalibState.recordedPoses || autoCalibState.recordedPoses.length < 3) {
        addLog('warning', `⚠️ 位姿数据不足，至少需要3个，当前只有${autoCalibState.recordedPoses?.length || 0}个`);
        return;
    }
    
    const poseCount = autoCalibState.recordedPoses.length;
    
    const config = {
        version: '4.0',
        calibrationType: 'eye-in-hand',
        calibrationMethod: 'pose-based-auto',
        savedAt: new Date().toISOString(),
        recordedPoses: autoCalibState.recordedPoses
    };
    
    const dataStr = JSON.stringify(config, null, 2);
    const blob = new Blob([dataStr], {type: 'application/json'});
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `auto_hand_eye_poses_${Date.now()}.json`;
    a.click();
    URL.revokeObjectURL(url);
    
    addLog('success', `💾 已保存配置：${poseCount}个记录的位姿（v4.0格式）`);
}

// 从文件加载运动数据
function handleLoadAllPosesFromFile() {
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
                
                // 检查版本和数据结构
                if (config.version === '4.0' && config.recordedPoses) {
                    // 新版本：记录的位姿列表
                    if (!Array.isArray(config.recordedPoses) || config.recordedPoses.length < 3) {
                        addLog('warning', `⚠️ 配置文件中的位姿数据不足（${config.recordedPoses?.length || 0}个），至少需要3个`);
                        return;
                    }
                    
                    autoCalibState.recordedPoses = config.recordedPoses;
                    autoCalibState.totalSteps = config.recordedPoses.length;
                    autoCalibState.collectedCalibrationData = [];
                    
                    addLog('success', `✅ 已加载配置：${config.recordedPoses.length}个记录的位姿`);
                    addLog('info', '💡 现在可以开始自动标定');
                    
                } else if (config.version === '3.0' && config.motionGroups) {
                    // 旧版本：运动组数据（兼容处理）
                    addLog('info', '📋 检测到v3.0格式配置文件（运动组格式）');
                    addLog('info', '🔄 正在转换为v4.0格式...');
                    
                    // 从motionGroups中提取所有机器人位姿
                    const extractedPoses = [];
                    config.motionGroups.forEach((group, groupIdx) => {
                        if (group.pose1 && group.pose1.robot_pose) {
                            extractedPoses.push({
                                position: group.pose1.robot_pose.position,
                                orientation: group.pose1.robot_pose.orientation
                            });
                        }
                        if (group.pose2 && group.pose2.robot_pose) {
                            extractedPoses.push({
                                position: group.pose2.robot_pose.position,
                                orientation: group.pose2.robot_pose.orientation
                            });
                        }
                    });
                    
                    if (extractedPoses.length < 3) {
                        addLog('error', `❌ 提取的位姿数量不足（${extractedPoses.length}个），至少需要3个`);
        return;
    }
    
                    // 转换为v4.0格式
                    autoCalibState.recordedPoses = extractedPoses;
                    autoCalibState.totalSteps = extractedPoses.length;
                    autoCalibState.collectedCalibrationData = [];
                    
                    addLog('success', `✅ 已转换并加载：${extractedPoses.length}个位姿（从${config.motionGroups.length}个运动组）`);
                    addLog('info', '💡 现在可以开始自动标定，系统将重新采集标定板数据');
        } else {
                    addLog('error', '❌ 文件格式错误：不支持的版本或缺少必要字段');
        return;
    }
                    
                    // 更新UI显示
                updateRecordedPosesDisplay();
                updateAutoCalibrationProgress();
                updateAutoStatusCard();
                
            } catch (error) {
                addLog('error', `❌ 解析文件失败：${error.message}`);
            }
        };
        reader.readAsText(file);
    };
    
    input.click();
}

// 注意：旧的角点法相关函数已移除，当前只使用recordedPoses流程

// ============= 自动标定核心功能 =============

// 等待机器人运动完成

// 移动机器人到指定位姿
async function moveRobotToPose(targetPose, useJoints = false, velocityFactor = 0.5, accelerationFactor = 0.5) {
    try {
        addLog('info', `🤖 正在移动机器人到位姿...`);
        addLog('info', `   位置: X=${targetPose.position.x.toFixed(3)}m, Y=${targetPose.position.y.toFixed(3)}m, Z=${targetPose.position.z.toFixed(3)}m`);
        if (targetPose.orientation) {
            addLog('info', `   姿态(四元数): x=${targetPose.orientation.x.toFixed(4)}, y=${targetPose.orientation.y.toFixed(4)}, z=${targetPose.orientation.z.toFixed(4)}, w=${targetPose.orientation.w.toFixed(4)}`);
        } else {
            addLog('warning', `   ⚠️  警告：位姿数据缺少orientation（旋转信息）！`);
        }
        
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
async function captureImageAndExtractCorners(poseIndex) {
    try {
        // 步骤1: 拍照
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
        
        // 步骤2: 等待图像可用（优化：减少轮询间隔和最大尝试次数）
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
        
        // 步骤3: 提取标定板位姿
        addLog('info', `   🔍 提取标定板位姿...`);
        const squareSize = parseFloat(document.getElementById('auto-checkerboard-size').value) || 15;
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
        
        // 显示带角点的图像
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
        
        // 获取当前机器人位姿（运动后的实际位姿）
        const robotResponse = await fetch('/api/robot_status');
        if (!robotResponse.ok) {
            throw new Error('获取机器人位姿失败');
        }
        
        const robotData = await robotResponse.json();
        if (!robotData.success || !robotData.is_online) {
            throw new Error('机器人未在线');
        }
        
        // 返回采集的数据（包含角点信息）
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
                orientation: boardPoseData.orientation
            },
            corners: boardPoseData.corners || [],  // 角点像素坐标
            corners_3d: boardPoseData.corners_3d || []  // 角点相机坐标（如果有）
        };
        
    } catch (error) {
        addLog('error', `❌ 采集图像和角点失败：${error.message}`);
        throw error;
    }
}

// 开始自动标定（完全自动化流程）
async function handleAutoCalibStart() {
    try {
        if (autoCalibState.isRunning) {
            addLog('warning', '⚠️ 自动标定正在进行中');
            return;
        }
        
        // 检查记录的位姿数量（v4.0格式）
        if (!autoCalibState.recordedPoses || autoCalibState.recordedPoses.length < 3) {
            addLog('error', `❌ 位姿数据不足，至少需要3个位姿，当前只有${autoCalibState.recordedPoses?.length || 0}个`);
            addLog('info', '💡 请先点击"记录机器人位姿"记录多个位姿（建议5-8个）');
            return;
        }
        
        if (!cameraParamsLoaded) {
            addLog('warning', '⚠️ 请先加载相机参数（系统将自动从ROS2话题获取）');
            await new Promise(resolve => setTimeout(resolve, 1000));  // 优化：从2000ms减少到1000ms
            if (!cameraParamsLoaded) {
                addLog('error', '❌ 未能从ROS2话题获取相机参数，请检查相机是否在线');
                return;
            }
        }
        
        // 初始化状态
        autoCalibState.isRunning = true;
        autoCalibState.currentStep = 0;
        autoCalibState.totalSteps = autoCalibState.recordedPoses.length;
        autoCalibState.collectedCalibrationData = [];
        updateAutoCalibrationDataDisplay(null);  // 清空数据显示
        
        addLog('info', '════════════════════════════════════════════');
        addLog('info', `🚀 开始完全自动手眼标定`);
        addLog('info', `📋 位姿数量: ${autoCalibState.totalSteps}个`);
        addLog('info', `📋 标定方法: 姿态法（AX=XB）`);
        addLog('info', '════════════════════════════════════════════');
        
        // 更新进度条
        updateAutoCalibrationProgress();
        
        // 依次处理每个位姿
        for (let i = 0; i < autoCalibState.recordedPoses.length && autoCalibState.isRunning; i++) {
            const poseIndex = i + 1;
            const targetPose = autoCalibState.recordedPoses[i];
            
            autoCalibState.currentStep = poseIndex;
            updateAutoCalibrationProgress();
            
            addLog('info', `\n📍 [${poseIndex}/${autoCalibState.totalSteps}] 处理位姿 #${poseIndex}`);
            addLog('info', `   目标位置: X=${targetPose.position.x.toFixed(3)}m, Y=${targetPose.position.y.toFixed(3)}m, Z=${targetPose.position.z.toFixed(3)}m`);
            
            try {
                // 步骤1: 移动机器人到目标位姿
                addLog('info', `   步骤1/3: 移动机器人到位姿 #${poseIndex}...`);
                await moveRobotToPose(targetPose, false, 0.2, 0.1);
                
                // 步骤2: 采集图像和角点
                addLog('info', `   步骤2/3: 采集图像和角点...`);
                const calibrationData = await captureImageAndExtractCorners(poseIndex);
                
                // 步骤3: 保存采集的数据
                autoCalibState.collectedCalibrationData.push(calibrationData);
                
                // 更新数据显示（每个位姿到位后更新一次）
                updateAutoCalibrationDataDisplay();
                
                addLog('success', `✅ 位姿 #${poseIndex} 数据采集完成`);
                
            } catch (error) {
                addLog('error', `❌ 位姿 #${poseIndex} 处理失败：${error.message}`);
                addLog('warning', '⚠️ 跳过当前位姿，继续下一个');
                continue;
            }
        }
        
        if (!autoCalibState.isRunning) {
            addLog('warning', '⚠️ 自动标定已停止');
            return;
        }
        
        // 检查采集的数据是否足够
        if (autoCalibState.collectedCalibrationData.length < 2) {
            addLog('error', `❌ 有效数据不足，至少需要2组，当前只有${autoCalibState.collectedCalibrationData.length}组`);
            autoCalibState.isRunning = false;
            return;
        }
        
        // 步骤4: 执行标定计算
        addLog('info', '\n════════════════════════════════════════════');
        addLog('info', `📊 开始标定计算（使用${autoCalibState.collectedCalibrationData.length}组数据）...`);
        
        await performAutoCalibrationFromCollectedData();
        
        autoCalibState.isRunning = false;
        updateAutoCalibrationProgress();
        
        addLog('success', '🎉 自动标定流程完成！');
        
    } catch (error) {
        addLog('error', `❌ 自动标定失败：${error.message}`);
        console.error('自动标定失败:', error);
        autoCalibState.isRunning = false;
        updateAutoCalibrationProgress();
    }
}

// 使用采集的数据执行标定计算
async function performAutoCalibrationFromCollectedData() {
    try {
        // 将采集的数据转换为运动组格式（用于AX=XB方法）
        // 每两个相邻的位姿组成一个运动组
        const motionGroups = [];
        
        for (let i = 0; i < autoCalibState.collectedCalibrationData.length - 1; i++) {
            const pose1Data = autoCalibState.collectedCalibrationData[i];
            const pose2Data = autoCalibState.collectedCalibrationData[i + 1];
            
            motionGroups.push({
                pose1: {
                    robot_pos_x: pose1Data.robot_pose.position.x,
                    robot_pos_y: pose1Data.robot_pose.position.y,
                    robot_pos_z: pose1Data.robot_pose.position.z,
                    robot_ori_x: pose1Data.robot_pose.orientation.x,
                    robot_ori_y: pose1Data.robot_pose.orientation.y,
                    robot_ori_z: pose1Data.robot_pose.orientation.z,
                    robot_ori_w: pose1Data.robot_pose.orientation.w
                },
                pose2: {
                    robot_pos_x: pose2Data.robot_pose.position.x,
                    robot_pos_y: pose2Data.robot_pose.position.y,
                    robot_pos_z: pose2Data.robot_pose.position.z,
                    robot_ori_x: pose2Data.robot_pose.orientation.x,
                    robot_ori_y: pose2Data.robot_pose.orientation.y,
                    robot_ori_z: pose2Data.robot_pose.orientation.z,
                    robot_ori_w: pose2Data.robot_pose.orientation.w
                },
                board_pose1: {
                    position: pose1Data.board_pose.position,
                    orientation: pose1Data.board_pose.orientation
                },
                board_pose2: {
                    position: pose2Data.board_pose.position,
                    orientation: pose2Data.board_pose.orientation
                }
            });
        }
        
        if (motionGroups.length < 2) {
            throw new Error(`运动组数据不足，至少需要2组，当前只有${motionGroups.length}组`);
        }
        
        const calibData = {
            calibration_type: 'eye-in-hand',
            calibration_method: 'pose-based',
            motion_groups: motionGroups
        };
        
        addLog('info', `📊 提交标定数据：${motionGroups.length}组运动数据`);
        
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
            // 保存标定结果
            autoCalibState.calibrationResult = result;
            
            // 更新数据显示（不清空，保持所有位姿数据）
            updateAutoCalibrationDataDisplay();
            
            addLog('success', `✅ 标定成功！`);
            addLog('info', `📊 标定方法：${result.method || 'Pose-Based (AX=XB)'}`);
            
            if (result.evaluation) {
                addLog('info', `📊 标定误差统计：`);
                addLog('info', `   平均误差: ${result.evaluation.mean_error.toFixed(3)} mm`);
                addLog('info', `   最大误差: ${result.evaluation.max_error.toFixed(3)} mm`);
                addLog('info', `   最小误差: ${result.evaluation.min_error.toFixed(3)} mm`);
                addLog('info', `   标准差: ${result.evaluation.std_error.toFixed(3)} mm`);
            }
            
            updateAutoStatusCard();
        } else {
            addLog('error', `❌ 标定失败：${result.error || '未知错误'}`);
        }
        
    } catch (error) {
        addLog('error', `❌ 标定计算失败：${error.message}`);
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
function updateAutoCalibrationDataDisplay() {
    const displayDiv = document.getElementById('auto-calibration-data-display');
    if (!displayDiv) return;
    
    // 获取已收集的数据列表
    const collectedData = autoCalibState.collectedCalibrationData || [];
    
    if (collectedData.length === 0) {
        displayDiv.innerHTML = '<div style="text-align: center; color: #999; font-size: 15px; padding: 40px 20px;">🤖 等待自动标定开始，数据将显示在这里...</div>';
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
    
    html += '<div style="overflow-x: auto;">';
    html += '<table class="data-table">';
    
    // 表头
    html += '<thead>';
    html += '<tr>';
    html += '<th style="min-width: 60px;">位姿</th>';
    html += '<th style="min-width: 80px;">角点数</th>';
    html += '<th style="min-width: 100px;">机器人 X (mm)</th>';
    html += '<th style="min-width: 100px;">机器人 Y (mm)</th>';
    html += '<th style="min-width: 100px;">机器人 Z (mm)</th>';
    html += '<th style="min-width: 100px;">机器人 Qx</th>';
    html += '<th style="min-width: 100px;">机器人 Qy</th>';
    html += '<th style="min-width: 100px;">机器人 Qz</th>';
    html += '<th style="min-width: 100px;">机器人 Qw</th>';
    html += '<th style="min-width: 100px;">标定板 X (mm)</th>';
    html += '<th style="min-width: 100px;">标定板 Y (mm)</th>';
    html += '<th style="min-width: 100px;">标定板 Z (mm)</th>';
    html += '<th style="min-width: 100px;">标定板 Qx</th>';
    html += '<th style="min-width: 100px;">标定板 Qy</th>';
    html += '<th style="min-width: 100px;">标定板 Qz</th>';
    html += '<th style="min-width: 100px;">标定板 Qw</th>';
    html += '</tr>';
    html += '</thead>';
    
    // 表格内容
    html += '<tbody>';
    collectedData.forEach((data, index) => {
        html += '<tr>';
        html += `<td class="pose-number">#${data.pose_index || (index + 1)}</td>`;
        html += `<td>${data.corners?.length || 0}</td>`;
        
        // 机器人位姿
        if (data.robot_pose) {
            html += `<td class="data-value">${(data.robot_pose.position.x * 1000).toFixed(3)}</td>`;
            html += `<td class="data-value">${(data.robot_pose.position.y * 1000).toFixed(3)}</td>`;
            html += `<td class="data-value">${(data.robot_pose.position.z * 1000).toFixed(3)}</td>`;
            html += `<td class="data-value">${data.robot_pose.orientation.x.toFixed(6)}</td>`;
            html += `<td class="data-value">${data.robot_pose.orientation.y.toFixed(6)}</td>`;
            html += `<td class="data-value">${data.robot_pose.orientation.z.toFixed(6)}</td>`;
            html += `<td class="data-value">${data.robot_pose.orientation.w.toFixed(6)}</td>`;
        } else {
            html += '<td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td>';
        }
        
        // 标定板位姿
        if (data.board_pose) {
            html += `<td class="data-value">${data.board_pose.position.x.toFixed(3)}</td>`;
            html += `<td class="data-value">${data.board_pose.position.y.toFixed(3)}</td>`;
            html += `<td class="data-value">${data.board_pose.position.z.toFixed(3)}</td>`;
            if (data.board_pose.orientation) {
                html += `<td class="data-value">${data.board_pose.orientation.x.toFixed(6)}</td>`;
                html += `<td class="data-value">${data.board_pose.orientation.y.toFixed(6)}</td>`;
                html += `<td class="data-value">${data.board_pose.orientation.z.toFixed(6)}</td>`;
                html += `<td class="data-value">${data.board_pose.orientation.w.toFixed(6)}</td>`;
            } else {
                html += '<td>-</td><td>-</td><td>-</td><td>-</td>';
            }
        } else {
            html += '<td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td><td>-</td>';
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
function showCalibrationResultModal() {
    const modal = document.getElementById('calibration-result-modal');
    const content = document.getElementById('calibration-result-content');
    
    if (!modal || !content) {
        addLog('error', '❌ 无法显示标定结果：模态框元素不存在');
        return;
    }
    
    const result = autoCalibState.calibrationResult;
    
    if (!result || !result.success) {
        addLog('error', '❌ 没有可用的标定结果，请先执行标定');
        return;
    }
    
    // 构建结果显示HTML - 使用现代化的卡片式设计
    let html = '<style>';
    html += '.calib-result-card { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 20px; border-radius: 12px; margin-bottom: 20px; box-shadow: 0 8px 16px rgba(0,0,0,0.1); }';
    html += '.calib-info-card { background: white; border: 1px solid #e0e0e0; border-radius: 10px; padding: 18px; margin-bottom: 18px; box-shadow: 0 2px 8px rgba(0,0,0,0.05); transition: transform 0.2s, box-shadow 0.2s; }';
    html += '.calib-info-card:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(0,0,0,0.1); }';
    html += '.calib-stat-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin: 15px 0; }';
    html += '.calib-stat-item { background: linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%); padding: 14px; border-radius: 8px; text-align: center; border-left: 4px solid #667eea; }';
    html += '.calib-stat-value { font-size: 1.4em; font-weight: bold; margin-top: 5px; }';
    html += '.calib-table { width: 100%; border-collapse: separate; border-spacing: 0; border-radius: 8px; overflow: hidden; box-shadow: 0 2px 8px rgba(0,0,0,0.05); }';
    html += '.calib-table thead { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; }';
    html += '.calib-table th { padding: 12px; text-align: left; font-weight: 600; }';
    html += '.calib-table td { padding: 10px 12px; border-bottom: 1px solid #f0f0f0; }';
    html += '.calib-table tbody tr:hover { background: #f8f9fa; }';
    html += '.calib-table tbody tr:last-child td { border-bottom: none; }';
    html += '.calib-badge { display: inline-block; padding: 4px 10px; border-radius: 12px; font-size: 0.85em; font-weight: 600; }';
    html += '.calib-badge-success { background: #d4edda; color: #155724; }';
    html += '.calib-badge-warning { background: #fff3cd; color: #856404; }';
    html += '.calib-badge-danger { background: #f8d7da; color: #721c24; }';
    html += '.calib-code-block { background: #1e1e1e; color: #d4d4d4; padding: 15px; border-radius: 8px; overflow-x: auto; font-family: "Consolas", "Monaco", monospace; font-size: 0.9em; line-height: 1.6; }';
    html += '.calib-details { margin: 10px 0; }';
    html += '.calib-details summary { cursor: pointer; padding: 12px; background: #f8f9fa; border-radius: 6px; font-weight: 600; color: #495057; transition: background 0.2s; }';
    html += '.calib-details summary:hover { background: #e9ecef; }';
    html += '.calib-details-content { padding: 15px; background: #f8f9fa; border-radius: 0 0 6px 6px; margin-top: 5px; }';
    html += '.calib-highlight-box { padding: 15px; border-radius: 8px; margin: 10px 0; border-left: 4px solid; }';
    html += '.calib-highlight-info { background: #e7f3ff; border-color: #0066cc; }';
    html += '.calib-highlight-warning { background: #fff3cd; border-color: #ffc107; }';
    html += '.calib-section-title { font-size: 1.2em; font-weight: 700; color: #333; margin: 20px 0 15px 0; padding-bottom: 10px; border-bottom: 2px solid #667eea; }';
    html += '.calib-description { background: #f0f7ff; border-left: 4px solid #0066cc; padding: 12px 15px; border-radius: 6px; margin: 12px 0; font-size: 0.9em; color: #495057; line-height: 1.6; }';
    html += '.calib-description strong { color: #0066cc; }';
    html += '.calib-help-text { font-size: 0.85em; color: #6c757d; margin-top: 8px; font-style: italic; }';
    html += '</style>';
    
    html += '<div style="font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif; line-height: 1.6; color: #333;">';
    
    // 基本信息 - 使用渐变卡片
    html += '<div class="calib-result-card">';
    html += '<h3 style="margin: 0 0 15px 0; font-size: 1.5em; display: flex; align-items: center; gap: 10px;">';
    html += '<span style="font-size: 1.2em;">✅</span> 标定成功';
    html += '</h3>';
    html += '<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin-top: 15px;">';
    html += `<div><strong style="opacity: 0.9;">标定方法：</strong><div style="margin-top: 5px; font-size: 1.1em;">${result.method || 'Pose-Based (AX=XB)'}</div></div>`;
    html += `<div><strong style="opacity: 0.9;">标定类型：</strong><div style="margin-top: 5px; font-size: 1.1em;">${result.calibration_type || 'Eye-in-Hand'}</div></div>`;
    html += `<div><strong style="opacity: 0.9;">标定时间：</strong><div style="margin-top: 5px; font-size: 1.1em;">${new Date().toLocaleString('zh-CN')}</div></div>`;
    html += '</div>';
    html += '<div class="calib-description" style="margin-top: 15px; background: rgba(255,255,255,0.2); border-left-color: rgba(255,255,255,0.5);">';
    html += '<strong>💡 说明：</strong>手眼标定用于确定相机坐标系与机器人末端坐标系之间的变换关系。标定结果包含旋转矩阵和平移向量，可用于将相机坐标系下的点转换到机器人末端坐标系。';
    html += '</div>';
    html += '</div>';
    
    // 误差统计（增强版）
    if (result.evaluation) {
        html += '<div class="calib-info-card">';
        html += '<div class="calib-section-title">📊 标定误差统计</div>';
        html += '<div class="calib-description">';
        html += '<strong>误差含义：</strong>标定误差反映了标定结果的精度。误差越小，标定质量越高。';
        html += '<ul style="margin: 8px 0 0 20px; padding: 0;">';
        html += '<li><strong>平均误差：</strong>所有数据点的平均位置误差，是标定精度的主要指标</li>';
        html += '<li><strong>最大/最小误差：</strong>误差的极值，反映标定结果的稳定性</li>';
        html += '<li><strong>标准差：</strong>误差的离散程度，越小表示标定结果越稳定</li>';
        html += '</ul>';
        html += '<div class="calib-help-text">💡 通常平均误差 < 2mm 为优秀，< 5mm 为良好</div>';
        html += '</div>';
        
        const meanError = result.evaluation.mean_error;
        const errorColor = meanError < 1.0 ? '#28a745' : meanError < 5.0 ? '#ffc107' : '#dc3545';
        const errorBadgeClass = meanError < 1.0 ? 'calib-badge-success' : meanError < 5.0 ? 'calib-badge-warning' : 'calib-badge-danger';
        const errorLevel = meanError < 1.0 ? '优秀' : meanError < 5.0 ? '良好' : '需改进';
        
        html += '<div class="calib-stat-grid">';
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">平均误差</div><div class="calib-stat-value" style="color: ${errorColor};">${meanError.toFixed(3)} mm</div><div style="margin-top: 5px;"><span class="calib-badge ${errorBadgeClass}">${errorLevel}</span></div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">最大误差</div><div class="calib-stat-value">${result.evaluation.max_error.toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">最小误差</div><div class="calib-stat-value" style="color: #28a745;">${result.evaluation.min_error.toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">标准差</div><div class="calib-stat-value">${result.evaluation.std_error.toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">数据组数</div><div class="calib-stat-value">${result.evaluation.data_count || autoCalibState.collectedCalibrationData?.length || 'N/A'}</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">变异系数</div><div class="calib-stat-value">${(result.evaluation.std_error / meanError * 100).toFixed(2)}%</div></div>`;
        html += '</div>';
        
        // 误差分位数统计
        if (result.evaluation.errors_per_motion && result.evaluation.errors_per_motion.length > 0) {
            const percentiles = calculatePercentiles(result.evaluation.errors_per_motion);
            html += '<details class="calib-details"><summary>📈 误差分布统计</summary>';
            html += '<div class="calib-details-content">';
            html += '<div class="calib-stat-grid">';
            html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">中位数 (P50)</div><div class="calib-stat-value">${percentiles.p50.toFixed(3)} mm</div></div>`;
            html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">第一四分位 (P25)</div><div class="calib-stat-value">${percentiles.p25.toFixed(3)} mm</div></div>`;
            html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">第三四分位 (P75)</div><div class="calib-stat-value">${percentiles.p75.toFixed(3)} mm</div></div>`;
            html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">90%分位 (P90)</div><div class="calib-stat-value">${percentiles.p90.toFixed(3)} mm</div></div>`;
            html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">95%分位 (P95)</div><div class="calib-stat-value">${percentiles.p95.toFixed(3)} mm</div></div>`;
            html += '</div></div></details>';
            
            // 每个运动组的详细误差
            html += '<details class="calib-details"><summary>📋 各运动组误差详情</summary>';
            html += '<div class="calib-details-content" style="max-height: 300px; overflow-y: auto;">';
            html += '<table class="calib-table">';
            html += '<thead><tr><th>运动组</th><th>误差 (mm)</th><th>状态</th></tr></thead>';
            html += '<tbody>';
            result.evaluation.errors_per_motion.forEach((error, idx) => {
                const errorColor = error < 1.0 ? '#28a745' : error < 5.0 ? '#ffc107' : '#dc3545';
                const badgeClass = error < 1.0 ? 'calib-badge-success' : error < 5.0 ? 'calib-badge-warning' : 'calib-badge-danger';
                const status = error < 1.0 ? '✅ 优秀' : error < 5.0 ? '⚠️ 良好' : '❌ 需改进';
                html += `<tr><td><strong>#${idx + 1}</strong></td>`;
                html += `<td style="color: ${errorColor}; font-weight: 600;">${error.toFixed(3)}</td>`;
                html += `<td><span class="calib-badge ${badgeClass}">${status}</span></td></tr>`;
            });
            html += '</tbody></table></div></details>';
        }
        html += '</div>';
        
        // 运动质量评估（增强版）
        if (result.evaluation.avg_motion_quality !== undefined) {
            const qualityPercent = (result.evaluation.avg_motion_quality * 100).toFixed(1);
            const qualityColor = result.evaluation.avg_motion_quality >= 0.8 ? '#28a745' : 
                               (result.evaluation.avg_motion_quality >= 0.6 ? '#ffc107' : '#dc3545');
            const qualityBadgeClass = result.evaluation.avg_motion_quality >= 0.8 ? 'calib-badge-success' : 
                                    (result.evaluation.avg_motion_quality >= 0.6 ? 'calib-badge-warning' : 'calib-badge-danger');
            
            html += '<div class="calib-info-card">';
            html += '<div class="calib-section-title">⭐ 运动质量评估</div>';
            html += `<div style="text-align: center; padding: 20px; background: linear-gradient(135deg, ${qualityColor}15 0%, ${qualityColor}05 100%); border-radius: 8px; margin: 15px 0;">`;
            html += `<div style="font-size: 0.9em; opacity: 0.8; margin-bottom: 10px;">平均质量</div>`;
            html += `<div style="font-size: 2.5em; font-weight: bold; color: ${qualityColor};">${qualityPercent}%</div>`;
            html += `<span class="calib-badge ${qualityBadgeClass}" style="margin-top: 10px; display: inline-block;">`;
            html += qualityPercent >= 80 ? '优秀' : (qualityPercent >= 60 ? '良好' : '需改进');
            html += '</span></div>';
            
            if (result.evaluation.motion_quality_analysis && result.evaluation.motion_quality_analysis.length > 0) {
                html += '<details class="calib-details"><summary>查看详细质量分析</summary>';
                html += '<div class="calib-details-content" style="max-height: 350px; overflow-y: auto;">';
                html += '<table class="calib-table">';
                html += '<thead><tr><th>运动组</th><th>质量</th><th>平移 (mm)</th><th>旋转 (°)</th><th>反馈</th></tr></thead>';
                html += '<tbody>';
                result.evaluation.motion_quality_analysis.forEach((motion, idx) => {
                    const motionColor = motion.quality_score >= 0.8 ? '#28a745' : 
                                      (motion.quality_score >= 0.6 ? '#ffc107' : '#dc3545');
                    const motionBadgeClass = motion.quality_score >= 0.8 ? 'calib-badge-success' : 
                                           (motion.quality_score >= 0.6 ? 'calib-badge-warning' : 'calib-badge-danger');
                    html += `<tr>`;
                    html += `<td><strong>#${idx + 1}</strong></td>`;
                    html += `<td><span class="calib-badge ${motionBadgeClass}">${(motion.quality_score * 100).toFixed(1)}%</span></td>`;
                    html += `<td>${motion.translation_mm ? motion.translation_mm.toFixed(1) : 'N/A'}</td>`;
                    html += `<td>${motion.rotation_deg ? motion.rotation_deg.toFixed(1) : 'N/A'}</td>`;
                    html += `<td style="font-size: 0.9em;">${motion.feedback || ''}</td>`;
                    html += `</tr>`;
                });
                html += '</tbody></table></div></details>';
            }
            html += '</div>';
        }
    }
    
    // 变换矩阵
    if (result.transformation_matrix) {
        html += '<div class="calib-info-card">';
        html += '<div class="calib-section-title">🔢 完整变换矩阵 (4×4)</div>';
        html += '<div class="calib-description">';
        html += '<strong>变换矩阵含义：</strong>完整的4×4齐次变换矩阵，包含旋转和平移信息，可直接用于坐标变换。';
        html += '<div class="calib-help-text">💡 矩阵格式：前3×3为旋转矩阵，第4列前3行为平移向量，最后一行通常为[0, 0, 0, 1]</div>';
        html += '</div>';
        html += '<details class="calib-details"><summary style="font-size: 1.05em;">📋 变换矩阵 (相机坐标系 → 机器人末端坐标系) - 点击展开</summary>';
        html += '<div class="calib-details-content">';
        html += '<div class="calib-code-block">';
        result.transformation_matrix.forEach((row, i) => {
            html += `[${row.map(v => v.toFixed(6).padStart(10)).join(', ')}]`;
            if (i < result.transformation_matrix.length - 1) html += '\n';
        });
        html += '</div>';
        html += '<div class="calib-help-text" style="margin-top: 10px;">使用此矩阵可将相机坐标系下的点P_cam转换为机器人末端坐标系下的点P_gripper：P_gripper = T × P_cam</div>';
        html += '</div></details>';
        html += '</div>';
    }
    
    // 旋转矩阵和平移向量（增强版）
    if (result.rotation_matrix && result.translation_vector) {
        html += '<div class="calib-info-card">';
        html += '<div class="calib-section-title">📐 旋转表示</div>';
        html += '<div class="calib-description">';
        html += '<strong>旋转矩阵含义：</strong>描述相机坐标系相对于机器人末端坐标系的旋转关系。';
        html += '<div class="calib-help-text">💡 旋转矩阵是3×3的正交矩阵，行列式为1，表示纯旋转（无缩放）</div>';
        html += '</div>';
        
        // 旋转矩阵
        html += '<details class="calib-details"><summary style="font-size: 1.05em;">📋 旋转矩阵 (3×3) - 点击展开</summary>';
        html += '<div class="calib-details-content">';
        html += '<div class="calib-code-block">';
        result.rotation_matrix.forEach((row, i) => {
            html += `[${row.map(v => v.toFixed(6).padStart(10)).join(', ')}]`;
            if (i < result.rotation_matrix.length - 1) html += '\n';
        });
        html += '</div>';
        html += '<div class="calib-help-text" style="margin-top: 10px;">矩阵的每一行表示相机坐标系的X、Y、Z轴在机器人末端坐标系下的方向向量</div>';
        html += '</div></details>';
        
        // 欧拉角表示
        try {
            const euler = rotationMatrixToEulerAngles(result.rotation_matrix);
            html += '<div class="calib-highlight-box calib-highlight-info" style="margin-top: 15px;">';
            html += '<div style="font-weight: 600; margin-bottom: 10px; color: #0066cc; font-size: 1.05em;">🎯 欧拉角表示 (ZYX顺序，单位：度)</div>';
            html += '<div class="calib-help-text" style="margin-bottom: 10px;">欧拉角是旋转的直观表示，按Z→Y→X顺序依次旋转</div>';
            html += '<div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-top: 10px;">';
            html += `<div style="text-align: center; padding: 12px; background: white; border-radius: 6px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);"><div style="font-size: 0.85em; opacity: 0.7; margin-bottom: 5px;">Roll (绕X轴)</div><div style="font-size: 1.4em; font-weight: bold; color: #0066cc;">${euler.roll.toFixed(3)}°</div></div>`;
            html += `<div style="text-align: center; padding: 12px; background: white; border-radius: 6px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);"><div style="font-size: 0.85em; opacity: 0.7; margin-bottom: 5px;">Pitch (绕Y轴)</div><div style="font-size: 1.4em; font-weight: bold; color: #0066cc;">${euler.pitch.toFixed(3)}°</div></div>`;
            html += `<div style="text-align: center; padding: 12px; background: white; border-radius: 6px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);"><div style="font-size: 0.85em; opacity: 0.7; margin-bottom: 5px;">Yaw (绕Z轴)</div><div style="font-size: 1.4em; font-weight: bold; color: #0066cc;">${euler.yaw.toFixed(3)}°</div></div>`;
            html += '</div></div>';
        } catch (e) {
            console.warn('计算欧拉角失败:', e);
        }
        
        // 轴角表示
        try {
            const axisAngle = rotationMatrixToAxisAngle(result.rotation_matrix);
            html += '<div class="calib-highlight-box calib-highlight-warning" style="margin-top: 15px;">';
            html += '<div style="font-weight: 600; margin-bottom: 10px; color: #856404; font-size: 1.05em;">🔄 轴角表示</div>';
            html += '<div class="calib-help-text" style="margin-bottom: 10px;">轴角表示：绕指定轴旋转指定角度，是旋转的另一种表示方式</div>';
            html += `<div style="margin-top: 10px; padding: 10px; background: white; border-radius: 6px;"><strong>旋转轴（单位向量）：</strong><code style="background: #f8f9fa; padding: 5px 10px; border-radius: 4px; font-size: 0.95em; display: inline-block; margin-left: 8px;">[${axisAngle.axis.map(v => v.toFixed(4)).join(', ')}]</code></div>`;
            html += `<div style="margin-top: 10px; padding: 10px; background: white; border-radius: 6px;"><strong>旋转角度：</strong><span style="font-size: 1.4em; font-weight: bold; color: #856404; margin-left: 8px;">${axisAngle.angle.toFixed(3)}°</span></div>`;
            html += '</div>';
        } catch (e) {
            console.warn('计算轴角失败:', e);
        }
        html += '</div>';
        
        // 平移向量（增强版）
        const translation = result.translation_vector;
        const translationMag = vectorMagnitude(translation);
        html += '<div class="calib-info-card">';
        html += '<div class="calib-section-title">📍 平移向量</div>';
        html += '<div class="calib-description">';
        html += '<strong>平移向量含义：</strong>表示相机坐标系原点在机器人末端坐标系下的位置（单位：毫米）。';
        html += '<div class="calib-help-text">💡 平移向量描述了相机相对于机器人末端的空间位置关系</div>';
        html += '</div>';
        html += `<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin: 15px 0;">`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">X分量</div><div class="calib-stat-value">${translation[0].toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">Y分量</div><div class="calib-stat-value">${translation[1].toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">Z分量</div><div class="calib-stat-value">${translation[2].toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item" style="border-left-color: #0066cc;"><div style="font-size: 0.9em; opacity: 0.8;">模长（距离）</div><div class="calib-stat-value" style="color: #0066cc; font-size: 1.5em;">${translationMag.toFixed(3)} mm</div></div>`;
        html += `</div>`;
        html += `<div class="calib-highlight-box calib-highlight-info" style="margin-top: 15px;">`;
        html += `<div style="margin-bottom: 10px; padding: 10px; background: white; border-radius: 6px;"><strong>完整向量值：</strong><code style="background: #f8f9fa; padding: 5px 10px; border-radius: 4px; font-size: 0.95em; display: inline-block; margin-left: 8px;">[${translation.map(v => v.toFixed(3)).join(', ')}]</code></div>`;
        html += `<div style="padding: 10px; background: white; border-radius: 6px;"><strong>方向单位向量：</strong><code style="background: #f8f9fa; padding: 5px 10px; border-radius: 4px; font-size: 0.95em; display: inline-block; margin-left: 8px;">[${translation.map(v => (v / translationMag).toFixed(4)).join(', ')}]</code></div>`;
        html += `</div></div>`;
    }
    
    html += '</div>';
    
    content.innerHTML = html;
    modal.style.display = 'block';
    
    // 绑定确认保存按钮
    const confirmBtn = document.getElementById('btn-confirm-save-calibration');
    if (confirmBtn) {
        confirmBtn.onclick = () => {
            closeCalibrationResultModal();
            doSaveCalibration();
        };
    }
}

// 关闭标定结果模态框
function closeCalibrationResultModal() {
    const modal = document.getElementById('calibration-result-modal');
    if (modal) {
        modal.style.display = 'none';
    }
}

// 实际执行保存操作
async function doSaveCalibration() {
    addLog('info', '💾 正在保存标定结果...');
    
    try {
        // 调用后端API生成标定结果XML
        const response = await fetch('/api/hand_eye/save_calibration', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({
                filename: `hand_eye_calibration_${Date.now()}.xml`
            })
        });
        
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        
        const result = await response.json();
        
        if (result.success && result.xml_content) {
            // 创建Blob并下载
            const blob = new Blob([result.xml_content], {type: 'application/xml'});
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `hand_eye_calibration_${Date.now()}.xml`;
            a.click();
            URL.revokeObjectURL(url);
            
            addLog('success', '✅ 标定结果已保存到文件');
            addLog('info', '   文件格式: XML');
            addLog('info', '   包含内容: 变换矩阵、标定误差、运动质量分析');
        } else {
            addLog('error', `❌ 保存失败：${result.error || '未知错误'}`);
        }
        
    } catch (error) {
        addLog('error', `❌ 保存失败：${error.message}`);
        console.error('保存标定结果失败:', error);
    }
}

// 保存标定结果（先显示确认）
async function handleSaveCalibrationForAuto() {
    // 检查是否有标定结果
    if (!autoCalibState.calibrationResult || !autoCalibState.calibrationResult.success) {
        addLog('error', '❌ 没有可用的标定结果，请先执行标定');
        return;
    }
    
    // 显示标定结果确认模态框
    showCalibrationResultModal();
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

// 更新智能操作状态卡片
function updateAutoStatusCard() {
    const statusCard = document.getElementById('auto-status-card');
    const statusText = document.getElementById('auto-status-text');
    const statusIcon = document.getElementById('auto-status-icon');
    const nextStep = document.getElementById('auto-next-step');
    
    if (!statusCard || !statusText || !statusIcon || !nextStep) return;
    
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

