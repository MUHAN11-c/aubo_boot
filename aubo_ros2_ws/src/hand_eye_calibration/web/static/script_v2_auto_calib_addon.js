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
function initAutoCalibButtons() {    // 姿态法数据采集按钮
    const btnRecordRobotPose = document.getElementById('btn-auto-record-robot-pose');
    const btnClearMotionData = document.getElementById('btn-auto-clear-motion-data');
    const btnSaveAllPoses = document.getElementById('btn-auto-save-all-poses');
    const btnLoadAllPoses = document.getElementById('btn-auto-load-all-poses');    if (btnRecordRobotPose) {
        btnRecordRobotPose.addEventListener('click', () => {            handleRecordRobotPose();
        });    }
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

// ============= 位姿管理功能 =============

// 获取当前机器人位姿
async function handleGetCurrentPose() {
    addLog('info', '📡 正在获取机器人当前位姿...');
    
    try {
        const response = await fetch('/api/robot_status');
        
        if (!response.ok) {
            addLog('error', `❌ 获取失败：HTTP ${response.status}`);
            return;
        }
        
        const data = await response.json();
        
        if (!data.success) {
            addLog('error', `❌ 获取失败：${data.message || '未知错误'}`);
            return;
        }
        
        if (!data.is_online) {
            addLog('warning', '⚠️ 机器人未在线');
            return;
        }
        
        // 保存当前位姿
        autoCalibState.currentRobotPose = {
            position: data.cartesian_position.position,
            orientation: data.cartesian_position.orientation
        };
        
        addLog('success', '✅ 已获取当前位姿');
        addLog('info', `   位置: X=${data.cartesian_position.position.x.toFixed(3)}, Y=${data.cartesian_position.position.y.toFixed(3)}, Z=${data.cartesian_position.position.z.toFixed(3)}`);
        addLog('info', `   姿态: X=${data.cartesian_position.orientation.x.toFixed(3)}, Y=${data.cartesian_position.orientation.y.toFixed(3)}, Z=${data.cartesian_position.orientation.z.toFixed(3)}, W=${data.cartesian_position.orientation.w.toFixed(3)}`);
        
        // 更新UI显示
        updateCalibrationPoseDisplay(autoCalibState.currentRobotPose, false);
        
    } catch (error) {
        addLog('error', `❌ 请求失败：${error.message}`);
        console.error('获取机器人位姿失败:', error);
    }
}

// 保存为标定位姿
function handleSaveCalibrationPose() {
    if (!autoCalibState.currentRobotPose) {
        addLog('warning', '⚠️ 请先点击"获取当前位姿"');
        return;
    }
    
    // 保存为标定位姿
    autoCalibState.savedCalibrationPose = JSON.parse(JSON.stringify(autoCalibState.currentRobotPose));
    
    addLog('success', '✅ 已保存为标定位姿');
    addLog('info', '💡 自动标定将使用此位姿作为基准');
    
    // 更新UI显示
    updateCalibrationPoseDisplay(autoCalibState.savedCalibrationPose, true);
}

// 清除标定位姿
function handleClearCalibrationPose() {
    autoCalibState.savedCalibrationPose = null;
    autoCalibState.currentRobotPose = null;
    
    addLog('info', '🗑️ 已清除标定位姿');
    
    // 更新UI显示
    const infoDiv = document.getElementById('auto-calibration-pose-info');
    if (infoDiv) {
        infoDiv.innerHTML = `
            <p style="color: #6c757d; margin: 0;">暂无保存的标定位姿</p>
            <p style="color: #6c757d; margin: 5px 0 0 0; font-size: 11px;">提示：手动移动机器人到标定位置，点击"获取当前位姿"→"保存为标定位姿"</p>
        `;
    }
}

// ============= 姿态法标定功能 =============

// 开始新运动组
function handleStartMotionGroup() {
    if (autoCalibState.currentMotionGroup && 
        (!autoCalibState.currentMotionGroup.pose2 || 
         !autoCalibState.currentMotionGroup.pose2.board_pose)) {
        addLog('warning', '⚠️ 当前运动组未完成，请先完成当前组的采集');
        return;
    }
    
    // 创建新的运动组
    autoCalibState.currentMotionGroup = {
        pose1: {robot_pose: null, board_pose: null},
        pose2: {robot_pose: null, board_pose: null}
    };
    
    addLog('info', `🔄 开始新运动组 #${autoCalibState.motionGroups.length + 1}`);
    addLog('info', '💡 步骤：1) 移动到位置1 → 记录机器人位姿 → 拍照并提取标定板位姿');
    addLog('info', '💡      2) 移动到位置2 → 记录机器人位姿 → 拍照并提取标定板位姿');
    
    // 更新UI显示
    updateMotionProgress();
}

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
        await sleep(500);
        
        // 等待图像可用并显示
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

// 拍照并提取标定板位姿
async function handleCaptureAndExtractBoard() {
    if (!autoCalibState.currentMotionGroup) {
        addLog('warning', '⚠️ 请先点击"开始新运动组"');
        return;
    }
    
    // 确定记录到pose1还是pose2
    let targetPose = null;
    let poseName = '';
    
    if (!autoCalibState.currentMotionGroup.pose1.robot_pose) {
        addLog('warning', '⚠️ 请先记录位置1的机器人位姿');
        return;
    } else if (!autoCalibState.currentMotionGroup.pose1.board_pose) {
        targetPose = autoCalibState.currentMotionGroup.pose1;
        poseName = '位置1';
    } else if (!autoCalibState.currentMotionGroup.pose2.robot_pose) {
        addLog('warning', '⚠️ 请先记录位置2的机器人位姿');
        return;
    } else if (!autoCalibState.currentMotionGroup.pose2.board_pose) {
        targetPose = autoCalibState.currentMotionGroup.pose2;
        poseName = '位置2';
    } else {
        addLog('warning', '⚠️ 当前运动组已完成，请开始新运动组');
        return;
    }
    
    addLog('info', `📷 正在拍照并提取${poseName}的标定板位姿...`);
    
    try {
        // 步骤1: 触发拍照
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
        
        addLog('success', '✅ 拍照完成');
        await sleep(500);
        
        // 步骤2: 等待图像可用
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
            throw new Error('获取图像超时');
        }
        
        // 显示图像
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
            };
            
            imgElement.onerror = function() {
                addLog('error', '❌ 图像加载失败');
            };
            
            // 设置图像源（base64数据不需要时间戳）
            imgElement.src = imageData.image;
        }
        
        // 步骤3: 提取标定板位姿
        addLog('info', '🔍 提取标定板位姿...');
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
        
        // 保存标定板位姿
        targetPose.board_pose = {
            position: boardPoseData.position,
            orientation: boardPoseData.orientation
        };
        
        addLog('success', `✅ 已提取${poseName}的标定板位姿`);
        addLog('info', `   位置: X=${boardPoseData.position.x.toFixed(2)}, Y=${boardPoseData.position.y.toFixed(2)}, Z=${boardPoseData.position.z.toFixed(2)}`);
        
        // 检查是否完成当前组
        if (poseName === '位置1') {
            addLog('info', '💡 下一步：移动到位置2 → 记录机器人位姿 → 拍照并提取标定板位姿');
        } else {
            // 完成一组运动
            autoCalibState.motionGroups.push(JSON.parse(JSON.stringify(autoCalibState.currentMotionGroup)));
            const groupCount = autoCalibState.motionGroups.length;
            
            addLog('success', `🎉 运动组 #${groupCount} 采集完成！`);
            addLog('info', `💡 当前共有 ${groupCount} 组运动数据（至少需要2组，建议3-5组）`);
            
            if (groupCount < 2) {
                addLog('info', '💡 继续采集下一组运动数据');
            } else {
                addLog('info', '💡 现在可以开始自动标定或继续采集更多组数据');
            }
            
            // 清空当前运动组
            autoCalibState.currentMotionGroup = null;
        }
        
        // 更新UI显示
        updateMotionProgress();
        
    } catch (error) {
        addLog('error', `❌ ${error.message}`);
        console.error('拍照并提取标定板位姿失败:', error);
    }
}

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
            <div style="padding: 8px; background: white; border-radius: 4px; text-align: center; color: #666; font-size: 11px;">
                暂无记录的位姿，请点击"记录机器人位姿"按钮
            </div>
        `;
    } else {
        let html = '';
        
        autoCalibState.recordedPoses.forEach((pose, index) => {
            const poseNum = index + 1;
            html += `
                <div style="padding: 10px; margin-bottom: 8px; background: white; border: 1px solid #e0e0e0; border-radius: 4px; font-size: 11px;">
                    <div style="font-weight: bold; color: #1976d2; margin-bottom: 6px;">
                        📍 位姿 #${poseNum}
                    </div>
                    <div style="color: #666; line-height: 1.6;">
                        <strong>位置:</strong> X=${pose.position.x.toFixed(3)}m, Y=${pose.position.y.toFixed(3)}m, Z=${pose.position.z.toFixed(3)}m<br>
                        <strong>姿态:</strong> X=${pose.orientation.x.toFixed(3)}, Y=${pose.orientation.y.toFixed(3)}, Z=${pose.orientation.z.toFixed(3)}, W=${pose.orientation.w.toFixed(3)}
                    </div>
                </div>
            `;
        });
        
        listDiv.innerHTML = html;
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

// 保存记录的位姿到文件
function handleSaveAllPosesToFile() {
    const poseCount = autoCalibState.recordedPoses?.length || 0;
    
    if (poseCount < 3) {
        addLog('warning', `⚠️ 位姿数据不足，至少需要3个，当前只有${poseCount}个`);
        return;
    }
    
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
    
    addLog('success', `💾 已保存配置：${poseCount}个记录的位姿`);
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
                    // 旧版本：运动组数据（兼容）
                    addLog('warning', '⚠️ 检测到旧版本配置文件（运动组格式）');
                    addLog('warning', '⚠️ 当前系统使用新的位姿记录格式，请重新记录位姿');
                    return;
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

// 旧的角点法函数（已废弃，保留用于兼容）
async function handleGetShotPose() {
    addLog('info', '📡 正在获取拍照位姿...');
    
    try {
        const response = await fetch('/api/robot_status');
        
        if (!response.ok) {
            addLog('error', `❌ 获取失败：HTTP ${response.status}`);
            return;
        }
        
        const data = await response.json();
        
        if (!data.success) {
            addLog('error', `❌ 获取失败：${data.message || '未知错误'}`);
            return;
        }
        
        if (!data.is_online) {
            addLog('warning', '⚠️ 机器人未在线');
            return;
        }
        
        // 保存当前拍照位姿（临时）
        autoCalibState.currentShotPose = {
            position: data.cartesian_position.position,
            orientation: data.cartesian_position.orientation
        };
        
        addLog('success', '✅ 已获取拍照位姿');
        addLog('info', `   位置: X=${data.cartesian_position.position.x.toFixed(3)}, Y=${data.cartesian_position.position.y.toFixed(3)}, Z=${data.cartesian_position.position.z.toFixed(3)}`);
        addLog('info', '💡 点击"添加到列表"保存此位姿');
        
    } catch (error) {
        addLog('error', `❌ 请求失败：${error.message}`);
        console.error('获取拍照位姿失败:', error);
    }
}

// 添加拍照位姿到列表
function handleAddShotPose() {
    if (!autoCalibState.currentShotPose) {
        addLog('warning', '⚠️ 请先点击"获取当前位姿"');
        return;
    }
    
    // 添加到拍照位姿列表
    const poseIndex = autoCalibState.shotPoses.length + 1;
    const pose = JSON.parse(JSON.stringify(autoCalibState.currentShotPose));
    pose.id = poseIndex;
    pose.name = `位姿${poseIndex}`;
    
    autoCalibState.shotPoses.push(pose);
    autoCalibState.currentShotPose = null;  // 清空临时位姿
    
    addLog('success', `✅ 已添加拍照位姿 #${poseIndex}`);
    addLog('info', `💡 当前共有 ${autoCalibState.shotPoses.length} 个拍照位姿（建议5-10个）`);
    
    // 更新UI显示
    updateShotPosesDisplay();
}

// 清空所有拍照位姿
function handleClearShotPoses() {
    const count = autoCalibState.shotPoses.length;
    autoCalibState.shotPoses = [];
    autoCalibState.currentShotPose = null;
    
    addLog('info', `🗑️ 已清除 ${count} 个拍照位姿`);
    
    // 更新UI显示
    updateShotPosesDisplay();
}

// 更新拍照位姿列表显示
function updateShotPosesDisplay() {
    const listDiv = document.getElementById('auto-shot-poses-list');
    const countSpan = document.getElementById('auto-shot-poses-count');
    
    if (countSpan) {
        countSpan.textContent = autoCalibState.shotPoses.length;
    }
    
    if (!listDiv) return;
    
    if (autoCalibState.shotPoses.length === 0) {
        listDiv.innerHTML = `
            <div style="padding: 8px; background: white; border-radius: 4px; text-align: center; color: #666; font-size: 11px;">
                暂无拍照位姿（建议添加 5-10 个）
            </div>
        `;
        return;
    }
    
    let html = '';
    autoCalibState.shotPoses.forEach((pose, idx) => {
        html += `
            <div style="padding: 6px 8px; margin-bottom: 4px; background: white; border-radius: 4px; border-left: 3px solid #2196f3; font-size: 11px;">
                <strong style="color: #1976d2;">#${pose.id} ${pose.name}</strong>
                <span style="color: #666; margin-left: 8px;">
                    X=${pose.position.x.toFixed(1)}, Y=${pose.position.y.toFixed(1)}, Z=${pose.position.z.toFixed(1)}
                </span>
                <button onclick="handleRemoveShotPose(${idx})" style="float: right; background: #d32f2f; color: white; border: none; padding: 2px 6px; border-radius: 3px; cursor: pointer; font-size: 10px;">删除</button>
            </div>
        `;
    });
    
    listDiv.innerHTML = html;
}

// 删除指定的拍照位姿
function handleRemoveShotPose(index) {
    if (index >= 0 && index < autoCalibState.shotPoses.length) {
        const removed = autoCalibState.shotPoses.splice(index, 1)[0];
        addLog('info', `🗑️ 已删除拍照位姿 #${removed.id}`);
        
        // 重新编号
        autoCalibState.shotPoses.forEach((pose, idx) => {
            pose.id = idx + 1;
            pose.name = `位姿${idx + 1}`;
        });
        
        updateShotPosesDisplay();
    }
}

// 采集图像并提取角点（用于点选）
async function handleCaptureForPick() {
    addLog('info', '📷 开始采集图像并提取角点...');
    
    try {
        // 步骤1: 触发拍照
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
        
        addLog('success', '✅ 拍照完成');
        await sleep(500);
        
        // 步骤2: 提取角点
        addLog('info', '🔍 提取棋盘格角点...');
        const squareSize = parseFloat(document.getElementById('auto-checkerboard-size').value) || 15;
        const cornersResponse = await fetch('/api/camera/extract_corners', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({square_size: squareSize})
        });
        
        if (!cornersResponse.ok) {
            throw new Error(`角点提取失败：HTTP ${cornersResponse.status}`);
        }
        
        const cornersData = await cornersResponse.json();
        if (!cornersData.success) {
            throw new Error(`角点提取失败：${cornersData.error || '未知错误'}`);
        }
        
        // 保存角点数据
        autoCalibState.cornerData = {
            corners: cornersData.corners_data,
            count: cornersData.corners_count,
            image: cornersData.image_with_corners
        };
        
        // 初始化点选数据
        autoCalibState.pickPoses = cornersData.corners_data.map((corner, idx) => ({
            corner_index: idx,
            corner_camera_coord: {
                x: corner.camera_x,
                y: corner.camera_y,
                z: corner.camera_z
            },
            robot_pose: null,  // 等待点选
            picked: false
        }));
        
        addLog('success', `✅ 检测到 ${cornersData.corners_count} 个角点`);
        addLog('info', '💡 现在可以逐个点选角点，记录机器人位姿');
        
        // 显示图像
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
            };
            
            imgElement.onerror = function() {
                addLog('error', '❌ 图像加载失败');
            };
            
            // 设置图像源（base64数据不需要时间戳）
            imgElement.src = cornersData.image_with_corners;
        }
        
        // 更新UI显示
        updatePickProgress();
        
    } catch (error) {
        addLog('error', `❌ ${error.message}`);
        console.error('采集角点失败:', error);
    }
}

// 记录当前角点的点选位姿
async function handleRecordPickPose() {
    if (!autoCalibState.cornerData) {
        addLog('warning', '⚠️ 请先采集图像并提取角点');
        return;
    }
    
    // 查找下一个未点选的角点
    const nextCorner = autoCalibState.pickPoses.find(p => !p.picked);
    if (!nextCorner) {
        addLog('warning', '⚠️ 所有角点已点选完毕');
        return;
    }
    
    addLog('info', `📡 正在记录角点 #${nextCorner.corner_index} 的位姿...`);
    
    try {
        const response = await fetch('/api/robot_status');
        
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        
        const data = await response.json();
        
        if (!data.success || !data.is_online) {
            throw new Error('机器人未在线');
        }
        
        // 保存点选位姿
        nextCorner.robot_pose = {
            position: data.cartesian_position.position,
            orientation: data.cartesian_position.orientation
        };
        nextCorner.picked = true;
        
        const pickedCount = autoCalibState.pickPoses.filter(p => p.picked).length;
        const totalCount = autoCalibState.pickPoses.length;
        
        addLog('success', `✅ 已记录角点 #${nextCorner.corner_index} (${pickedCount}/${totalCount})`);
        addLog('info', `   位置: X=${data.cartesian_position.position.x.toFixed(2)}, Y=${data.cartesian_position.position.y.toFixed(2)}, Z=${data.cartesian_position.position.z.toFixed(2)}`);
        
        if (pickedCount < totalCount) {
            addLog('info', `💡 请移动到下一个角点 #${nextCorner.corner_index + 1}`);
        } else {
            addLog('success', '🎉 所有角点已点选完成！');
            addLog('info', '💡 现在可以保存配置或开始自动标定');
        }
        
        // 更新UI显示
        updatePickProgress();
        
    } catch (error) {
        addLog('error', `❌ 获取位姿失败：${error.message}`);
    }
}

// 清除点选数据
function handleClearPickData() {
    autoCalibState.cornerData = null;
    autoCalibState.pickPoses = [];
    
    addLog('info', '🗑️ 已清除角点点选数据');
    
    // 更新UI显示
    updatePickProgress();
}

// 更新点选进度显示
function updatePickProgress() {
    const progressDiv = document.getElementById('auto-pick-progress');
    const listDiv = document.getElementById('auto-pick-poses-list');
    
    if (!progressDiv) return;
    
    if (!autoCalibState.cornerData) {
        progressDiv.innerHTML = '<span style="color: #666;">尚未采集角点数据</span>';
        if (listDiv) listDiv.style.display = 'none';
        return;
    }
    
    const pickedCount = autoCalibState.pickPoses.filter(p => p.picked).length;
    const totalCount = autoCalibState.pickPoses.length;
    const percentage = totalCount > 0 ? (pickedCount / totalCount * 100).toFixed(0) : 0;
    
    progressDiv.innerHTML = `
        <div style="margin-bottom: 6px;">
            <strong style="color: ${pickedCount === totalCount ? '#4caf50' : '#ff9800'};">
                ${pickedCount === totalCount ? '✅' : '📍'} 
                已点选: ${pickedCount}/${totalCount} (${percentage}%)
            </strong>
        </div>
        <div style="width: 100%; height: 8px; background: #e0e0e0; border-radius: 4px; overflow: hidden;">
            <div style="width: ${percentage}%; height: 100%; background: ${pickedCount === totalCount ? '#4caf50' : '#ff9800'}; transition: width 0.3s;"></div>
        </div>
        ${pickedCount < totalCount ? `<div style="margin-top: 4px; color: #ff9800; font-size: 10px;">💡 请移动机器人到角点 #${autoCalibState.pickPoses.find(p => !p.picked).corner_index}，然后点击"记录当前角点位姿"</div>` : ''}
    `;
    
    // 显示角点列表
    if (listDiv) {
        listDiv.style.display = 'block';
        let html = '<div style="display: grid; grid-template-columns: repeat(auto-fill, minmax(60px, 1fr)); gap: 4px; padding: 8px; background: white; border-radius: 4px;">';
        
        autoCalibState.pickPoses.forEach((p, idx) => {
            const bgColor = p.picked ? '#e8f5e9' : '#fff3e0';
            const borderColor = p.picked ? '#4caf50' : '#ff9800';
            const icon = p.picked ? '✅' : '⭕';
            html += `
                <div style="padding: 4px; background: ${bgColor}; border: 1px solid ${borderColor}; border-radius: 3px; text-align: center; font-size: 10px;">
                    ${icon} #${p.corner_index}
                </div>
            `;
        });
        
        html += '</div>';
        listDiv.innerHTML = html;
    }
}

// 保存所有位姿配置到文件（姿态法）
function handleSaveAllPosesToFile() {
    // 检查是否有未完成的当前组
    if (autoCalibState.currentMotionGroup) {
        const current = autoCalibState.currentMotionGroup;
        if (current.pose1.robot_pose && current.pose1.board_pose &&
            current.pose2.robot_pose && current.pose2.board_pose) {
            // 完成但未保存的组，自动保存
            autoCalibState.motionGroups.push(JSON.parse(JSON.stringify(current)));
            autoCalibState.currentMotionGroup = null;
            updateMotionProgress();
            addLog('info', '💡 自动保存了未完成的运动组');
        }
    }
    
    if (autoCalibState.motionGroups.length === 0) {
        addLog('warning', '⚠️ 请先采集运动数据（至少需要2组）');
        return;
    }
    
    const config = {
        version: '3.0',  // 姿态法版本
        calibrationType: 'eye-in-hand-pose-based',
        calibrationMethod: 'AX=XB',
        savedAt: new Date().toISOString(),
        motionGroups: autoCalibState.motionGroups  // 运动组数据
    };
    
    const dataStr = JSON.stringify(config, null, 2);
    const blob = new Blob([dataStr], {type: 'application/json'});
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `pose_based_motion_data_${Date.now()}.json`;
    a.click();
    URL.revokeObjectURL(url);
    
    addLog('success', `💾 已保存运动数据：${autoCalibState.motionGroups.length}个运动组`);
}

// 从文件加载位姿配置（姿态法）
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
                
                // 兼容处理：支持旧版本（v2.0角点法）和新版本（v3.0姿态法）
                if (config.version === '3.0' && config.motionGroups) {
                    // 新版本：姿态法数据
                    autoCalibState.motionGroups = config.motionGroups;
                    autoCalibState.currentMotionGroup = null;
                    
                    addLog('success', `✅ 已加载运动数据：${config.motionGroups.length}个运动组`);
                    addLog('info', '💡 现在可以开始自动标定');
                    
                    // 更新UI显示
                    updateMotionProgress();
                    
                } else if (config.shotPoses && config.pickPoses) {
                    // 旧版本：角点法数据（已弃用）
                    addLog('warning', '⚠️ 检测到旧版本配置文件（角点法）');
                    addLog('warning', '⚠️ 当前系统使用姿态法，无法加载角点法数据');
                    addLog('info', '💡 请重新采集姿态法运动数据');
                    
                } else {
                    addLog('error', '❌ 文件格式错误：缺少必要字段（motionGroups）');
                    return;
                }
                
            } catch (error) {
                addLog('error', `❌ 解析文件失败：${error.message}`);
            }
        };
        reader.readAsText(file);
    };
    
    input.click();
}

// 更新标定位姿显示
function updateCalibrationPoseDisplay(pose, isSaved) {
    if (!pose) return;
    
    const infoDiv = document.getElementById('auto-calibration-pose-info');
    if (!infoDiv) return;
    
    const statusColor = isSaved ? '#28a745' : '#17a2b8';
    const statusIcon = isSaved ? '💾' : '📡';
    const statusText = isSaved ? '已保存的标定位姿' : '当前获取的位姿（未保存）';
    
    infoDiv.innerHTML = `
        <p style="color: ${statusColor}; margin: 0; font-weight: bold;">${statusIcon} ${statusText}</p>
        <div style="margin-top: 8px;">
            <div style="color: #495057; margin: 4px 0;">
                <strong>位置:</strong> X=${pose.position.x.toFixed(3)}, Y=${pose.position.y.toFixed(3)}, Z=${pose.position.z.toFixed(3)}
            </div>
            <div style="color: #495057; margin: 4px 0;">
                <strong>姿态:</strong> X=${pose.orientation.x.toFixed(3)}, Y=${pose.orientation.y.toFixed(3)}, Z=${pose.orientation.z.toFixed(3)}, W=${pose.orientation.w.toFixed(3)}
            </div>
        </div>
        ${!isSaved ? '<p style="color: #ffc107; margin: 8px 0 0 0; font-size: 11px;">⚠️ 请点击"保存为标定位姿"按钮确认保存</p>' : ''}
    `;
}

// ============= 自动标定核心功能 =============

// 等待机器人运动完成
async function waitForRobotToStop(maxWaitTime = 30000) {
    const startTime = Date.now();
    while (Date.now() - startTime < maxWaitTime) {
        try {
            const response = await fetch('/api/robot_status');
            if (response.ok) {
                const data = await response.json();
                if (data.success && !data.in_motion) {
                    // 机器人已停止，再等待一小段时间确保稳定
                    await sleep(500);
                    return true;
                }
            }
        } catch (error) {
            console.error('检查机器人状态失败:', error);
        }
        await sleep(200);
    }
    return false;
}

// 移动机器人到指定位姿
async function moveRobotToPose(targetPose, useJoints = false, velocityFactor = 0.5, accelerationFactor = 0.5) {
    try {
        addLog('info', `🤖 正在移动机器人到位姿...`);
        addLog('info', `   位置: X=${targetPose.position.x.toFixed(3)}m, Y=${targetPose.position.y.toFixed(3)}m, Z=${targetPose.position.z.toFixed(3)}m`);
        
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
        
        // 等待机器人运动完成
        addLog('info', `⏳ 等待机器人运动到位...`);
        const stopped = await waitForRobotToStop(30000); // 最多等待30秒
        
        if (!stopped) {
            throw new Error('机器人运动超时');
        }
        
        addLog('success', `✅ 机器人已到达目标位姿`);
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
        await sleep(500);
        
        // 步骤2: 等待图像可用
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
        
        // 返回采集的数据
        return {
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
            }
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
        
        // 检查记录的位姿数量
        if (!autoCalibState.recordedPoses || autoCalibState.recordedPoses.length < 3) {
            addLog('error', `❌ 位姿数据不足，至少需要3个位姿，当前只有${autoCalibState.recordedPoses?.length || 0}个`);
            addLog('info', '💡 请先点击"记录机器人位姿"记录多个位姿（建议5-8个）');
            return;
        }
        
        if (!cameraParamsLoaded) {
            addLog('warning', '⚠️ 请先加载相机参数（系统将自动从ROS2话题获取）');
            await new Promise(resolve => setTimeout(resolve, 2000));
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
                await moveRobotToPose(targetPose, false, 0.5, 0.5);
                
                // 步骤2: 采集图像和角点
                addLog('info', `   步骤2/3: 采集图像和角点...`);
                const calibrationData = await captureImageAndExtractCorners(poseIndex);
                
                // 步骤3: 保存采集的数据
                autoCalibState.collectedCalibrationData.push(calibrationData);
                
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

// [已弃用] 暂停/继续/停止功能（旧的自动规划模式）
function handleAutoCalibPause() {
    // 已弃用：界面已简化，不再需要暂停功能
}

function handleAutoCalibResume() {
    // 已弃用：界面已简化，不再需要继续功能
}

// [已弃用] 停止自动标定
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

// ============= 多位姿自动标定执行 =============

// 执行多位姿Eye-in-Hand自动标定
async function executeMultiPoseAutoCalibration() {
    addLog('info', '════════════════════════════════════════════');
    addLog('info', '🔄 开始多位姿数据采集...');
    
    // 遍历所有拍照位姿
    for (let poseIdx = 0; poseIdx < autoCalibState.shotPoses.length && autoCalibState.isRunning; poseIdx++) {
        // 检查暂停状态
        while (autoCalibState.isPaused && autoCalibState.isRunning) {
            await sleep(500);
        }
        
        if (!autoCalibState.isRunning) break;
        
        const shotPose = autoCalibState.shotPoses[poseIdx];
        const stepNum = poseIdx + 1;
        
        addLog('info', `════════════════════════════════════════════`);
        addLog('info', `📍 [${stepNum}/${autoCalibState.totalSteps}] 拍照位姿 #${shotPose.id}`);
        addLog('info', `   位置: X=${shotPose.position.x.toFixed(1)}, Y=${shotPose.position.y.toFixed(1)}, Z=${shotPose.position.z.toFixed(1)}`);
        updateAutoCurrentStep(`位姿${stepNum}：准备移动...`);
        
        try {
            // TODO: 调用机器人移动API，移动到shotPose
            // await moveRobotToPose(shotPose);
            addLog('info', `🤖 需要手动移动机器人到位姿 #${shotPose.id}，然后继续`);
            addLog('warning', '⚠️ 自动移动功能未实现，请手动移动机器人到指定位姿后按"继续"');
            
            // 暂停等待用户手动移动
            autoCalibState.isPaused = true;
            const btnPause = document.getElementById('btn-auto-pause');
            const btnResume = document.getElementById('btn-auto-resume');
            if (btnPause) btnPause.style.display = 'none';
            if (btnResume) btnResume.style.display = 'inline-block';
            updateAutoStatus('等待手动移动', '#ffc107');
            
            // 等待用户点击继续
            while (autoCalibState.isPaused && autoCalibState.isRunning) {
                await sleep(500);
            }
            
            if (!autoCalibState.isRunning) break;
            
            // 等待机器人稳定
            updateAutoCurrentStep(`位姿${stepNum}：稳定中...`);
            await sleep(1000);
            
            // 采集图像
            updateAutoCurrentStep(`位姿${stepNum}：拍照...`);
            addLog('info', '📷 触发相机拍照...');
            
            const captureResponse = await fetch('/api/camera/capture', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'}
            });
            
            if (!captureResponse.ok) {
                throw new Error(`拍照失败：HTTP ${captureResponse.status}`);
            }
            
            const captureData = await captureResponse.json();
            if (!captureData.success) {
                throw new Error(captureData.error || '拍照失败');
            }
            
            addLog('success', '✅ 拍照完成');
            await sleep(500);
            
            // 获取图像
            updateAutoCurrentStep(`位姿${stepNum}：获取图像...`);
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
                throw new Error('获取图像超时');
            }
            
            addLog('success', '✅ 图像获取成功');
            
            // 提取角点
            updateAutoCurrentStep(`位姿${stepNum}：提取角点...`);
            addLog('info', '🔍 提取棋盘格角点...');
            
            const squareSize = parseFloat(document.getElementById('auto-checkerboard-size').value) || 15;
            const cornersResponse = await fetch('/api/camera/extract_corners', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({square_size: squareSize})
            });
            
            if (!cornersResponse.ok) {
                throw new Error(`角点提取失败：HTTP ${cornersResponse.status}`);
            }
            
            const cornersData = await cornersResponse.json();
            if (!cornersData.success) {
                throw new Error(cornersData.error || '角点提取失败');
            }
            
            addLog('success', `✅ 检测到 ${cornersData.corners_count} 个角点`);
            
            // 显示图像
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
                };
                
                imgElement.onerror = function() {
                    addLog('error', '❌ 图像加载失败');
                };
                
                // 设置图像源（base64数据不需要时间戳）
                imgElement.src = cornersData.image_with_corners;
            }
            
            // 保存数据
            updateAutoCurrentStep(`位姿${stepNum}：保存数据...`);
            autoCalibState.collectedData.push({
                shot_pose_id: shotPose.id,
                shot_pose: shotPose,
                corners: cornersData.corners_data,
                image: cornersData.image_with_corners
            });
            
            addLog('success', `✅ 位姿 #${shotPose.id} 数据采集完成`);
            
            // 更新进度
            autoCalibState.currentStep++;
            updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
            
            await sleep(500);
            
        } catch (error) {
            addLog('error', `❌ 位姿 #${shotPose.id} 采集失败：${error.message}`);
            addLog('warning', '⚠️ 跳过此位姿，继续下一个');
            autoCalibState.currentStep++;
            updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
        }
    }
    
    // 数据采集完成，执行标定
    if (autoCalibState.isRunning && autoCalibState.collectedData.length > 0) {
        addLog('info', `════════════════════════════════════════════`);
        addLog('success', `🎉 数据采集完成！共成功采集 ${autoCalibState.collectedData.length} 个位姿`);
        addLog('info', '🔄 开始执行手眼标定计算...');
        updateAutoCurrentStep('执行标定计算...');
        
        // 执行标定
        await performMultiPoseCalibration();
        
        // 标定完成
        autoCalibState.isRunning = false;
        updateAutoCalibButtons(false, false);
        
        if (autoCalibState.collectedData.length >= 3) {
            updateAutoStatus('已完成', '#28a745');
            updateAutoCurrentStep('标定完成');
        }
    } else if (autoCalibState.collectedData.length === 0) {
        addLog('error', '❌ 未采集到有效数据');
        autoCalibState.isRunning = false;
        updateAutoCalibButtons(false, false);
        updateAutoStatus('失败', '#dc3545');
    }
}

// ============= 智能角点选择 =============

// 评估角点在某个位姿下的质量
function evaluateCornerQuality(corner, poseIdx, allCollectedData) {
    let score = 1.0;
    const reasons = [];
    
    // 1. 深度一致性评分（如果多个位姿都有这个角点，计算深度方差）
    if (allCollectedData.length > 1) {
        const depths = [];
        allCollectedData.forEach(data => {
            const c = data.corners.find(c => c.index === corner.index);
            if (c && c.camera_z) {
                depths.push(c.camera_z);
            }
        });
        
        if (depths.length > 1) {
            const meanDepth = depths.reduce((a, b) => a + b, 0) / depths.length;
            const variance = depths.reduce((sum, d) => sum + Math.pow(d - meanDepth, 2), 0) / depths.length;
            const stdDev = Math.sqrt(variance);
            const depthConsistency = 1.0 - Math.min(stdDev / meanDepth, 0.1); // 标准差小于10%得满分
            score *= depthConsistency;
            reasons.push(`深度一致性: ${(depthConsistency * 100).toFixed(1)}%`);
        }
    }
    
    // 2. 视野中心度评分（距离图像中心越近越好）
    // 假设图像中心在 (320, 240)（需要从相机参数获取，这里简化处理）
    const imageCenterX = 320;
    const imageCenterY = 240;
    const pixelX = corner.pixel_x || 0;
    const pixelY = corner.pixel_y || 0;
    const distFromCenter = Math.sqrt(
        Math.pow(pixelX - imageCenterX, 2) + 
        Math.pow(pixelY - imageCenterY, 2)
    );
    const maxDist = Math.sqrt(Math.pow(imageCenterX, 2) + Math.pow(imageCenterY, 2));
    const centerScore = 1.0 - (distFromCenter / maxDist) * 0.3; // 最多扣30%
    score *= centerScore;
    reasons.push(`视野中心度: ${(centerScore * 100).toFixed(1)}%`);
    
    // 3. 深度合理性（太近或太远都不好）
    const depth = corner.camera_z || 0;
    if (depth > 200 && depth < 1000) {  // 200-1000mm范围内
        const depthScore = 1.0;
        score *= depthScore;
        reasons.push(`深度合理性: 100%`);
    } else {
        const depthScore = 0.8;  // 稍微扣分
        score *= depthScore;
        reasons.push(`深度合理性: 80%`);
    }
    
    return {
        score: score,
        reasons: reasons
    };
}

// 为每个角点选择最佳观测数据
function selectBestCornerObservations(allCollectedData) {
    // 构建角点索引到所有观测的映射
    const cornerObservations = {};
    
    allCollectedData.forEach((data, poseIdx) => {
        data.corners.forEach(corner => {
            if (!cornerObservations[corner.index]) {
                cornerObservations[corner.index] = [];
            }
            cornerObservations[corner.index].push({
                corner: corner,
                poseIdx: poseIdx,
                shotPose: autoCalibState.shotPoses[poseIdx]
            });
        });
    });
    
    // 为每个角点选择最佳观测
    const selectedObservations = {};
    const qualityStats = {
        totalCorners: Object.keys(cornerObservations).length,
        cornersWithMultipleObs: 0,
        averageQuality: 0
    };
    
    let totalQuality = 0;
    
    Object.keys(cornerObservations).forEach(cornerIdx => {
        const observations = cornerObservations[cornerIdx];
        
        if (observations.length > 1) {
            qualityStats.cornersWithMultipleObs++;
            
            // 评估每个观测的质量
            const observationsWithQuality = observations.map(obs => {
                const quality = evaluateCornerQuality(obs.corner, obs.poseIdx, allCollectedData);
                return {
                    ...obs,
                    quality: quality.score,
                    qualityReasons: quality.reasons
                };
            });
            
            // 选择质量最高的观测
            observationsWithQuality.sort((a, b) => b.quality - a.quality);
            selectedObservations[cornerIdx] = observationsWithQuality[0];
            totalQuality += observationsWithQuality[0].quality;
            
            if (observationsWithQuality.length > 1) {
                const best = observationsWithQuality[0];
                const second = observationsWithQuality[1];
                addLog('info', `📊 角点 #${cornerIdx}: 从${observations.length}个观测中选择位姿#${best.shotPose.id} (质量: ${(best.quality * 100).toFixed(1)}% vs ${(second.quality * 100).toFixed(1)}%)`);
            }
        } else {
            // 只有一个观测，直接使用
            selectedObservations[cornerIdx] = {
                ...observations[0],
                quality: 1.0,
                qualityReasons: ['唯一观测']
            };
            totalQuality += 1.0;
        }
    });
    
    qualityStats.averageQuality = totalQuality / qualityStats.totalCorners;
    
    addLog('info', `📊 智能角点选择完成：`);
    addLog('info', `   总角点数: ${qualityStats.totalCorners}`);
    addLog('info', `   多观测角点数: ${qualityStats.cornersWithMultipleObs}`);
    addLog('info', `   平均质量分数: ${(qualityStats.averageQuality * 100).toFixed(1)}%`);
    
    return selectedObservations;
}

// 执行姿态法标定计算
async function performPoseBasedCalibration() {
    try {
        addLog('info', '════════════════════════════════════════════');
        addLog('info', '🔄 开始姿态法标定计算（AX=XB方法）...');
        
        // 构建姿态法标定数据
        let motionGroups;
        try {
            motionGroups = autoCalibState.motionGroups.map((group, idx) => ({
                pose1: {
                    robot_pos_x: group.pose1.robot_pose.position.x,
                    robot_pos_y: group.pose1.robot_pose.position.y,
                    robot_pos_z: group.pose1.robot_pose.position.z,
                    robot_ori_x: group.pose1.robot_pose.orientation.x,
                    robot_ori_y: group.pose1.robot_pose.orientation.y,
                    robot_ori_z: group.pose1.robot_pose.orientation.z,
                    robot_ori_w: group.pose1.robot_pose.orientation.w
                },
                pose2: {
                    robot_pos_x: group.pose2.robot_pose.position.x,
                    robot_pos_y: group.pose2.robot_pose.position.y,
                    robot_pos_z: group.pose2.robot_pose.position.z,
                    robot_ori_x: group.pose2.robot_pose.orientation.x,
                    robot_ori_y: group.pose2.robot_pose.orientation.y,
                    robot_ori_z: group.pose2.robot_pose.orientation.z,
                    robot_ori_w: group.pose2.robot_pose.orientation.w
                },
                board_pose1: {
                    position: group.pose1.board_pose.position,
                    orientation: group.pose1.board_pose.orientation
                },
                board_pose2: {
                    position: group.pose2.board_pose.position,
                    orientation: group.pose2.board_pose.orientation
                }
            }));
        } catch (mapError) {
            addLog('error', `❌ 数据格式转换失败：${mapError.message}`);
            throw mapError;
        }
        
        const calibData = {
            calibration_type: 'eye-in-hand',
            calibration_method: 'pose-based',
            motion_groups: motionGroups
        };
        
        addLog('info', `📊 提交标定数据：${motionGroups.length}组运动数据`);
        addLog('info', `   每组包含：2个机器人位姿 + 2个标定板位姿`);
        
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
            // 保存标定结果到状态
            autoCalibState.calibrationResult = result;
            
            addLog('success', `✅ 标定成功！`);
            addLog('info', `📊 标定方法：${result.method || 'Pose-Based (AX=XB)'}`);
            
            if (result.evaluation) {
                addLog('info', `📊 标定误差统计：`);
                addLog('info', `   平均误差: ${result.evaluation.mean_error.toFixed(3)} mm`);
                addLog('info', `   最大误差: ${result.evaluation.max_error.toFixed(3)} mm`);
                addLog('info', `   最小误差: ${result.evaluation.min_error.toFixed(3)} mm`);
                addLog('info', `   标准差: ${result.evaluation.std_error.toFixed(3)} mm`);
                
                // 显示运动质量分析（借鉴MoveIt Calibration）
                if (result.evaluation.avg_motion_quality !== undefined) {
                    const qualityPercent = (result.evaluation.avg_motion_quality * 100).toFixed(1);
                    const qualityIcon = result.evaluation.avg_motion_quality >= 0.8 ? '⭐⭐⭐' : 
                                       (result.evaluation.avg_motion_quality >= 0.6 ? '⭐⭐' : '⭐');
                    addLog('info', `📊 运动质量评估：${qualityIcon} ${qualityPercent}%`);
                    
                    // 显示每个运动组的详细质量
                    if (result.evaluation.motion_quality_analysis) {
                        const lowQualityMotions = result.evaluation.motion_quality_analysis.filter(m => m.quality_score < 0.6);
                        if (lowQualityMotions.length > 0) {
                            addLog('warning', `⚠️ 发现${lowQualityMotions.length}个低质量运动组，建议重新采集`);
                        }
                    }
                }
            }
            
            updateAutoStatus('标定成功', '#28a745');
        } else {
            addLog('error', `❌ 标定失败：${result.error || '未知错误'}`);
            updateAutoStatus('标定失败', '#dc3545');
        }
        
    } catch (error) {
        addLog('error', `❌ 标定请求失败：${error.message}`);
        updateAutoStatus('标定失败', '#dc3545');
    }
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
        const currentPose = autoCalibState.savedCalibrationPose; // 使用保存的标定位姿
        
        addLog('info', `═══════════════════════════════════════`);
        addLog('info', `📍 [${stepNum}/${autoCalibState.totalSteps}] 开始采集第 ${stepNum} 次`);
        updateAutoCurrentStep(`第${stepNum}次：准备中...`);
        
        try {
            // 步骤1: 确认机器人位姿（不移动，仅提示）
            addLog('info', `🤖 使用标定位姿 (X:${currentPose.position.x.toFixed(3)}, Y:${currentPose.position.y.toFixed(3)}, Z:${currentPose.position.z.toFixed(3)})`);
            addLog('info', '💡 机器人保持在标定位姿，无需移动');
            
            // 等待机器人稳定（给用户时间调整，如果需要）
            updateAutoCurrentStep(`第${stepNum}次：稳定中...`);
            await sleep(1000);
            
            // 步骤2: 采集图像
            updateAutoCurrentStep(`第${stepNum}次：触发拍照...`);
            addLog('info', '📷 触发相机拍照...');
            
            const captureResponse = await fetch('/api/camera/capture', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'}
            });
            
            if (!captureResponse.ok) {
                addLog('error', `❌ 拍照失败：HTTP ${captureResponse.status}`);
                addLog('warning', '⚠️ 跳过当前采样，继续下一次');
                autoCalibState.currentStep++;
                updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
                continue;
            }
            
            const captureData = await captureResponse.json();
            
            if (!captureData.success) {
                addLog('error', `❌ 拍照失败：${captureData.error || '未知错误'}`);
                addLog('warning', '⚠️ 跳过当前采样，继续下一次');
                autoCalibState.currentStep++;
                updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
                continue;
            }
            
            addLog('success', '✅ 拍照命令已发送');
            updateAutoCurrentStep(`第${stepNum}次：获取图像...`);
            
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
                addLog('warning', '⚠️ 跳过当前采样，继续下一次');
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
            updateAutoCurrentStep(`第${stepNum}次：提取角点...`);
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
                addLog('warning', '⚠️ 跳过当前采样，继续下一次');
                autoCalibState.currentStep++;
                updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
                continue;
            }
            
            const cornersData = await cornersResponse.json();
            
            if (!cornersData.success) {
                addLog('error', `❌ 角点提取失败：${cornersData.error || '未知错误'}`);
                addLog('warning', '⚠️ 跳过当前采样，继续下一次');
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
            updateAutoCurrentStep(`第${stepNum}次：保存数据...`);
            autoCalibState.collectedData.push({
                group_id: stepNum,
                pose: currentPose,
                corners: cornersData.corners_data,
                image_with_corners: cornersData.image_with_corners
            });
            
            addLog('success', `✅ 第 ${stepNum} 次数据采集完成`);
            
            // 步骤5: 更新进度和表格
            autoCalibState.currentStep++;
            updateAutoProgress(autoCalibState.currentStep, autoCalibState.totalSteps);
            updateAutoPoseListTable();
            updateAutoPoseCount(autoCalibState.collectedData.length);
            
            // 短暂延时，让用户看到结果
            await sleep(800);
            
        } catch (error) {
            addLog('error', `❌ 执行失败：${error.message}`);
            addLog('warning', '⚠️ 跳过当前采样，继续下一次');
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
    html += '.calib-info-card { background: white; border: 1px solid #e0e0e0; border-radius: 10px; padding: 15px; margin-bottom: 15px; box-shadow: 0 2px 8px rgba(0,0,0,0.05); transition: transform 0.2s, box-shadow 0.2s; }';
    html += '.calib-info-card:hover { transform: translateY(-2px); box-shadow: 0 4px 12px rgba(0,0,0,0.1); }';
    html += '.calib-stat-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin: 15px 0; }';
    html += '.calib-stat-item { background: linear-gradient(135deg, #f5f7fa 0%, #c3cfe2 100%); padding: 12px; border-radius: 8px; text-align: center; border-left: 4px solid #667eea; }';
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
    html += '.calib-details summary { cursor: pointer; padding: 10px; background: #f8f9fa; border-radius: 6px; font-weight: 600; color: #495057; transition: background 0.2s; }';
    html += '.calib-details summary:hover { background: #e9ecef; }';
    html += '.calib-details-content { padding: 15px; background: #f8f9fa; border-radius: 0 0 6px 6px; margin-top: 5px; }';
    html += '.calib-highlight-box { padding: 15px; border-radius: 8px; margin: 10px 0; border-left: 4px solid; }';
    html += '.calib-highlight-info { background: #e7f3ff; border-color: #0066cc; }';
    html += '.calib-highlight-warning { background: #fff3cd; border-color: #ffc107; }';
    html += '.calib-section-title { font-size: 1.2em; font-weight: 700; color: #333; margin: 20px 0 15px 0; padding-bottom: 10px; border-bottom: 2px solid #667eea; }';
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
    html += '</div></div>';
    
    // 误差统计（增强版）
    if (result.evaluation) {
        html += '<div class="calib-info-card">';
        html += '<div class="calib-section-title">📊 标定误差统计</div>';
        
        const meanError = result.evaluation.mean_error;
        const errorColor = meanError < 1.0 ? '#28a745' : meanError < 5.0 ? '#ffc107' : '#dc3545';
        const errorBadgeClass = meanError < 1.0 ? 'calib-badge-success' : meanError < 5.0 ? 'calib-badge-warning' : 'calib-badge-danger';
        
        html += '<div class="calib-stat-grid">';
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">平均误差</div><div class="calib-stat-value" style="color: ${errorColor};">${meanError.toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">最大误差</div><div class="calib-stat-value">${result.evaluation.max_error.toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">最小误差</div><div class="calib-stat-value" style="color: #28a745;">${result.evaluation.min_error.toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">标准差</div><div class="calib-stat-value">${result.evaluation.std_error.toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">数据组数</div><div class="calib-stat-value">${result.evaluation.data_count || 'N/A'}</div></div>`;
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
        html += '<div class="calib-section-title">🔢 变换矩阵 (相机→末端执行器)</div>';
        html += '<div class="calib-code-block">';
        result.transformation_matrix.forEach((row, i) => {
            html += `[${row.map(v => v.toFixed(6).padStart(10)).join(', ')}]`;
            if (i < result.transformation_matrix.length - 1) html += '\n';
        });
        html += '</div></div>';
    }
    
    // 旋转矩阵和平移向量（增强版）
    if (result.rotation_matrix && result.translation_vector) {
        html += '<div class="calib-info-card">';
        html += '<div class="calib-section-title">📐 旋转表示</div>';
        
        // 旋转矩阵
        html += '<details class="calib-details"><summary>旋转矩阵 (3×3)</summary>';
        html += '<div class="calib-details-content"><div class="calib-code-block">';
        result.rotation_matrix.forEach((row, i) => {
            html += `[${row.map(v => v.toFixed(6).padStart(10)).join(', ')}]`;
            if (i < result.rotation_matrix.length - 1) html += '\n';
        });
        html += '</div></div></details>';
        
        // 欧拉角表示
        try {
            const euler = rotationMatrixToEulerAngles(result.rotation_matrix);
            html += '<div class="calib-highlight-box calib-highlight-info" style="margin-top: 15px;">';
            html += '<div style="font-weight: 600; margin-bottom: 10px; color: #0066cc;">🎯 欧拉角 (ZYX顺序，单位：度)</div>';
            html += '<div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-top: 10px;">';
            html += `<div style="text-align: center; padding: 10px; background: white; border-radius: 6px;"><div style="font-size: 0.85em; opacity: 0.7;">Roll (X轴)</div><div style="font-size: 1.3em; font-weight: bold; color: #0066cc;">${euler.roll.toFixed(3)}°</div></div>`;
            html += `<div style="text-align: center; padding: 10px; background: white; border-radius: 6px;"><div style="font-size: 0.85em; opacity: 0.7;">Pitch (Y轴)</div><div style="font-size: 1.3em; font-weight: bold; color: #0066cc;">${euler.pitch.toFixed(3)}°</div></div>`;
            html += `<div style="text-align: center; padding: 10px; background: white; border-radius: 6px;"><div style="font-size: 0.85em; opacity: 0.7;">Yaw (Z轴)</div><div style="font-size: 1.3em; font-weight: bold; color: #0066cc;">${euler.yaw.toFixed(3)}°</div></div>`;
            html += '</div></div>';
        } catch (e) {
            console.warn('计算欧拉角失败:', e);
        }
        
        // 轴角表示
        try {
            const axisAngle = rotationMatrixToAxisAngle(result.rotation_matrix);
            html += '<div class="calib-highlight-box calib-highlight-warning" style="margin-top: 15px;">';
            html += '<div style="font-weight: 600; margin-bottom: 10px; color: #856404;">🔄 轴角表示</div>';
            html += `<div style="margin-top: 10px;"><strong>旋转轴：</strong><code style="background: white; padding: 5px 10px; border-radius: 4px; font-size: 0.95em;">[${axisAngle.axis.map(v => v.toFixed(4)).join(', ')}]</code></div>`;
            html += `<div style="margin-top: 10px;"><strong>旋转角度：</strong><span style="font-size: 1.3em; font-weight: bold; color: #856404;">${axisAngle.angle.toFixed(3)}°</span></div>`;
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
        html += `<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin: 15px 0;">`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">X分量</div><div class="calib-stat-value">${translation[0].toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">Y分量</div><div class="calib-stat-value">${translation[1].toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item"><div style="font-size: 0.9em; opacity: 0.8;">Z分量</div><div class="calib-stat-value">${translation[2].toFixed(3)} mm</div></div>`;
        html += `<div class="calib-stat-item" style="border-left-color: #0066cc;"><div style="font-size: 0.9em; opacity: 0.8;">模长</div><div class="calib-stat-value" style="color: #0066cc;">${translationMag.toFixed(3)} mm</div></div>`;
        html += `</div>`;
        html += `<div class="calib-highlight-box calib-highlight-info" style="margin-top: 15px;">`;
        html += `<div style="margin-bottom: 8px;"><strong>向量值：</strong><code style="background: white; padding: 5px 10px; border-radius: 4px; font-size: 0.95em;">[${translation.map(v => v.toFixed(3)).join(', ')}]</code></div>`;
        html += `<div><strong>方向单位向量：</strong><code style="background: white; padding: 5px 10px; border-radius: 4px; font-size: 0.95em;">[${translation.map(v => (v / translationMag).toFixed(4)).join(', ')}]</code></div>`;
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

// ============= UI更新函数 =============

// [已弃用] UI更新函数（旧的自动规划模式）
function updateAutoCalibButtons(isRunning, isPaused) {
    // 已弃用：界面已简化，不再有这些按钮
}

function updateAutoStatus(text, color) {
    // 已弃用：界面已简化，不再有状态显示元素
}

function updateAutoProgress(current, total) {
    // 已弃用：界面已简化，不再有进度条
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

// 辅助函数：评估运动质量
function evaluateMotionQuality(motionGroup) {
    /**
     * 评估运动组的质量，提供实时反馈
     * 基于MoveIt Calibration的质量评估思想，简化实现
     */
    try {
        const pose1 = motionGroup.pose1.robot_pose.position;
        const pose2 = motionGroup.pose2.robot_pose.position;
        const board1 = motionGroup.pose1.board_pose.position;
        const board2 = motionGroup.pose2.board_pose.position;
        
        // 1. 计算机器人运动距离（平移）
        const dx = (pose2.x - pose1.x) * 1000;  // 转换为mm
        const dy = (pose2.y - pose1.y) * 1000;
        const dz = (pose2.z - pose1.z) * 1000;
        const robotTranslation = Math.sqrt(dx*dx + dy*dy + dz*dz);
        
        // 2. 计算标定板在相机中的运动距离
        const dbx = board2.x - board1.x;
        const dby = board2.y - board1.y;
        const dbz = board2.z - board1.z;
        const boardTranslation = Math.sqrt(dbx*dbx + dby*dby + dbz*dbz);
        
        // 3. 质量评分和反馈生成
        let score = 0;
        let feedback = [];
        let color = '#2196f3';
        
        // 评估机器人运动距离
        if (robotTranslation < 30) {
            feedback.push(`⚠️ 运动距离偏小(${robotTranslation.toFixed(1)}mm)，建议50-200mm`);
            score += 0.3;
            color = '#ff9800';
        } else if (robotTranslation >= 50 && robotTranslation <= 200) {
            feedback.push(`✅ 运动距离合适(${robotTranslation.toFixed(1)}mm)`);
            score += 1.0;
            color = '#4caf50';
        } else if (robotTranslation > 300) {
            feedback.push(`⚠️ 运动距离较大(${robotTranslation.toFixed(1)}mm)，注意标定板在视野内`);
            score += 0.7;
            color = '#ff9800';
        } else {
            feedback.push(`✓ 运动距离${robotTranslation.toFixed(1)}mm`);
            score += 0.8;
        }
        
        // 评估标定板在相机中的观测变化
        if (boardTranslation < 20) {
            feedback.push(`⚠️ 标定板在相机中移动较小(${boardTranslation.toFixed(1)}mm)`);
            score += 0.3;
            if (color === '#4caf50') color = '#ff9800';
        } else if (boardTranslation >= 50 && boardTranslation <= 300) {
            feedback.push(`✅ 标定板观测变化明显(${boardTranslation.toFixed(1)}mm)`);
            score += 1.0;
        } else {
            feedback.push(`✓ 标定板观测变化${boardTranslation.toFixed(1)}mm`);
            score += 0.8;
        }
        
        score = score / 2.0;  // 归一化到0-1
        
        // 生成质量图标
        let qualityIcon = '';
        if (score >= 0.8) {
            qualityIcon = '⭐⭐⭐';
        } else if (score >= 0.6) {
            qualityIcon = '⭐⭐';
        } else {
            qualityIcon = '⭐';
        }
        
        return {
            score: score,
            message: `${qualityIcon} 质量评分: ${(score * 100).toFixed(0)}% - ${feedback.join(', ')}`,
            color: color,
            robotTranslation: robotTranslation,
            boardTranslation: boardTranslation
        };
        
    } catch (error) {
        console.error('评估运动质量失败:', error);
        return {
            score: 0.5,
            message: '✓ 运动组已记录',
            color: '#2196f3',
            robotTranslation: 0,
            boardTranslation: 0
        };
    }
}

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

