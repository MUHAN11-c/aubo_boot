// ============================================================
// 自动手眼标定功能扩展 - 多位姿全自动版本
// Eye-in-Hand: 拍照位姿列表 + 角点点选数据（第一次设置，后续重复使用）
// ============================================================

// 自动标定全局状态
let autoCalibState = {
    isRunning: false,
    isPaused: false,
    currentStep: 0,
    totalSteps: 0,
    collectedData: [],
    calibrationType: 'eye-in-hand',
    
    // 多位姿管理（Eye-in-Hand全自动）
    shotPoses: [],  // 拍照位姿列表（多个，5-10个）
    currentRobotPose: null,  // 临时存储当前获取的位姿
    
    // 角点点选数据（仅需设置一次）
    corners: [],      // 角点列表 {index, camera_x, camera_y, camera_z}
    pickPoses: [],    // 每个角点对应的点选位姿
    pickImage: null,  // 用于点选的参考图像（带角点标记）
    currentPickIndex: 0  // 当前正在点选的角点索引
};

// 初始化自动标定选项卡功能
function initAutoHandEyeCalibTab() {
    console.log('初始化自动手眼标定功能（多位姿全自动版本）...');
    
    // 初始化按钮
    initAutoCalibButtons();
    
    // 初始化标定类型切换
    initAutoCalibTypeChange();
    
    addLog('success', '🤖 自动标定功能初始化完成（Eye-in-Hand多位姿模式）');
}

// 初始化按钮事件
function initAutoCalibButtons() {
    // 自动控制按钮
    const btnAutoStart = document.getElementById('btn-auto-start');
    const btnAutoPause = document.getElementById('btn-auto-pause');
    const btnAutoResume = document.getElementById('btn-auto-resume');
    const btnAutoStop = document.getElementById('btn-auto-stop');
    
    if (btnAutoStart) btnAutoStart.addEventListener('click', handleAutoCalibStart);
    if (btnAutoPause) btnAutoPause.addEventListener('click', handleAutoCalibPause);
    if (btnAutoResume) btnAutoResume.addEventListener('click', handleAutoCalibResume);
    if (btnAutoStop) btnAutoStop.addEventListener('click', handleAutoCalibStop);
    
    // 拍照位姿管理
    const btnGetShotPose = document.getElementById('btn-auto-get-shot-pose');
    const btnAddShotPose = document.getElementById('btn-auto-add-shot-pose');
    const btnClearShotPoses = document.getElementById('btn-auto-clear-shot-poses');
    
    if (btnGetShotPose) btnGetShotPose.addEventListener('click', handleGetShotPose);
    if (btnAddShotPose) btnAddShotPose.addEventListener('click', handleAddShotPose);
    if (btnClearShotPoses) btnClearShotPoses.addEventListener('click', handleClearShotPoses);
    
    // 角点点选管理
    const btnCaptureForPick = document.getElementById('btn-auto-capture-for-pick');
    const btnRecordPickPose = document.getElementById('btn-auto-record-pick-pose');
    const btnClearPickData = document.getElementById('btn-auto-clear-pick-data');
    
    if (btnCaptureForPick) btnCaptureForPick.addEventListener('click', handleCaptureForPick);
    if (btnRecordPickPose) btnRecordPickPose.addEventListener('click', handleRecordPickPose);
    if (btnClearPickData) btnClearPickData.addEventListener('click', handleClearPickData);
    
    // 位姿数据管理
    const btnSaveAllPoses = document.getElementById('btn-auto-save-all-poses');
    const btnLoadAllPoses = document.getElementById('btn-auto-load-all-poses');
    
    if (btnSaveAllPoses) btnSaveAllPoses.addEventListener('click', handleSaveAllPoses);
    if (btnLoadAllPoses) btnLoadAllPoses.addEventListener('click', handleLoadAllPoses);
    
    // 手动操作按钮
    const btnAutoLoadParams = document.getElementById('btn-auto-load-camera-params');
    const btnAutoCapture = document.getElementById('btn-auto-capture-image');
    const btnAutoExtract = document.getElementById('btn-auto-extract-corners');
    const btnAutoSavePose = document.getElementById('btn-auto-save-pose-data');
    const btnAutoLoadPose = document.getElementById('btn-auto-load-pose-data');
    const btnAutoStartCalib = document.getElementById('btn-auto-start-calibration');
    const btnAutoSaveCalib = document.getElementById('btn-auto-save-calibration');
    
    if (btnAutoLoadParams) btnAutoLoadParams.addEventListener('click', () => handleLoadCameraParamsForAuto());
    if (btnAutoCapture) btnAutoCapture.addEventListener('click', () => handleCaptureImageForAuto());
    if (btnAutoExtract) btnAutoExtract.addEventListener('click', () => handleExtractCornersForAuto());
    if (btnAutoSavePose) btnAutoSavePose.addEventListener('click', () => handleSavePoseDataForAuto());
    if (btnAutoLoadPose) btnAutoLoadPose.addEventListener('click', () => handleLoadPoseDataForAuto());
    if (btnAutoStartCalib) btnAutoStartCalib.addEventListener('click', () => handleStartCalibrationForAuto());
    if (btnAutoSaveCalib) btnAutoSaveCalib.addEventListener('click', () => handleSaveCalibrationForAuto());
}

// ============= 拍照位姿管理 =============

// 获取拍照位姿
async function handleGetShotPose() {
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
        
        // 保存当前位姿到临时变量
        autoCalibState.currentRobotPose = {
            position: data.cartesian_position.position,
            orientation: data.cartesian_position.orientation
        };
        
        addLog('success', '✅ 已获取当前位姿');
        addLog('info', `   位置: X=${data.cartesian_position.position.x.toFixed(3)}, Y=${data.cartesian_position.position.y.toFixed(3)}, Z=${data.cartesian_position.position.z.toFixed(3)}`);
        addLog('info', `   姿态: X=${data.cartesian_position.orientation.x.toFixed(3)}, Y=${data.cartesian_position.orientation.y.toFixed(3)}, Z=${data.cartesian_position.orientation.z.toFixed(3)}, W=${data.cartesian_position.orientation.w.toFixed(3)}`);
        
    } catch (error) {
        addLog('error', `❌ 请求失败：${error.message}`);
        console.error('获取机器人位姿失败:', error);
    }
}

// 添加到拍照位姿列表
function handleAddShotPose() {
    if (!autoCalibState.currentRobotPose) {
        addLog('warning', '⚠️ 请先点击"获取当前位姿"');
        return;
    }
    
    // 添加到列表
    autoCalibState.shotPoses.push(JSON.parse(JSON.stringify(autoCalibState.currentRobotPose)));
    
    addLog('success', `✅ 已添加第 ${autoCalibState.shotPoses.length} 个拍照位姿`);
    
    // 更新UI显示
    updateShotPosesList();
    
    // 清空临时变量
    autoCalibState.currentRobotPose = null;
}

// 清空拍照位姿列表
function handleClearShotPoses() {
    if (autoCalibState.shotPoses.length === 0) {
        addLog('info', '📝 拍照位姿列表已经是空的');
        return;
    }
    
    const count = autoCalibState.shotPoses.length;
    autoCalibState.shotPoses = [];
    autoCalibState.currentRobotPose = null;
    
    addLog('info', `🗑️ 已清空 ${count} 个拍照位姿`);
    
    // 更新UI
    updateShotPosesList();
}

// 更新拍照位姿列表显示
function updateShotPosesList() {
    const countSpan = document.getElementById('auto-shot-poses-count');
    const listDiv = document.getElementById('auto-shot-poses-list');
    
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
    autoCalibState.shotPoses.forEach((pose, index) => {
        html += `
            <div style="padding: 6px; margin-bottom: 4px; background: white; border-radius: 4px; display: flex; justify-content: space-between; align-items: center; font-size: 11px;">
                <span>
                    <strong style="color: #2196f3;">#${index + 1}</strong>
                    P(${pose.position.x.toFixed(1)}, ${pose.position.y.toFixed(1)}, ${pose.position.z.toFixed(1)})
                </span>
                <button onclick="removeShotPose(${index})" style="background: #f44336; color: white; border: none; padding: 2px 8px; border-radius: 3px; cursor: pointer; font-size: 10px;">删除</button>
            </div>
        `;
    });
    
    listDiv.innerHTML = html;
}

// 删除指定的拍照位姿
function removeShotPose(index) {
    if (index >= 0 && index < autoCalibState.shotPoses.length) {
        autoCalibState.shotPoses.splice(index, 1);
        addLog('info', `🗑️ 已删除拍照位姿 #${index + 1}`);
        updateShotPosesList();
    }
}

// ============= 角点点选管理 =============

// 采集图像并提取角点（用于点选）
async function handleCaptureForPick() {
    addLog('info', '📷 采集图像用于角点点选...');
    
    try {
        // 步骤1: 触发拍照
        const captureResponse = await fetch('/api/camera/capture', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'}
        });
        
        if (!captureResponse.ok) {
            throw new Error(`HTTP ${captureResponse.status}`);
        }
        
        const captureData = await captureResponse.json();
        if (!captureData.success) {
            throw new Error(captureData.error || '拍照失败');
        }
        
        addLog('success', '✅ 拍照成功，等待图像...');
        
        // 步骤2: 获取图像
        await sleep(1000);
        const imgResponse = await fetch('/api/current_image');
        const imgData = await imgResponse.json();
        
        if (!imgData.success || !imgData.image) {
            throw new Error('未能获取图像');
        }
        
        addLog('success', '✅ 图像获取成功');
        
        // 步骤3: 提取角点
        addLog('info', '🔍 正在提取角点...');
        const squareSize = parseFloat(document.getElementById('auto-checkerboard-size').value) || 15;
        
        const cornersResponse = await fetch('/api/camera/extract_corners', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({square_size: squareSize})
        });
        
        if (!cornersResponse.ok) {
            throw new Error(`HTTP ${cornersResponse.status}`);
        }
        
        const cornersData = await cornersResponse.json();
        
        if (!cornersData.success) {
            throw new Error(cornersData.error || '角点提取失败');
        }
        
        // 保存角点数据和图像
        autoCalibState.corners = cornersData.corners_data || [];
        autoCalibState.pickImage = cornersData.image_with_corners;
        autoCalibState.pickPoses = [];  // 清空旧的点选位姿
        autoCalibState.currentPickIndex = 0;
        
        addLog('success', `✅ 检测到 ${autoCalibState.corners.length} 个角点`);
        addLog('info', '💡 现在可以开始逐个点选角点了');
        
        // 显示图像
        displayPickImage();
        
        // 更新UI
        updatePickProgress();
        
    } catch (error) {
        addLog('error', `❌ 操作失败：${error.message}`);
        console.error('采集角点失败:', error);
    }
}

// 记录当前角点的点选位姿
async function handleRecordPickPose() {
    if (autoCalibState.corners.length === 0) {
        addLog('warning', '⚠️ 请先采集图像并提取角点');
        return;
    }
    
    if (autoCalibState.currentPickIndex >= autoCalibState.corners.length) {
        addLog('warning', '⚠️ 所有角点已完成点选');
        return;
    }
    
    addLog('info', '📡 正在获取当前角点的点选位姿...');
    
    try {
        const response = await fetch('/api/robot_status');
        
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        
        const data = await response.json();
        
        if (!data.success || !data.is_online) {
            throw new Error('机器人未在线');
        }
        
        const pickPose = {
            position: data.cartesian_position.position,
            orientation: data.cartesian_position.orientation
        };
        
        // 保存点选位姿
        autoCalibState.pickPoses.push(pickPose);
        
        const corner = autoCalibState.corners[autoCalibState.currentPickIndex];
        addLog('success', `✅ 已记录角点 #${corner.index} 的点选位姿 (${autoCalibState.currentPickIndex + 1}/${autoCalibState.corners.length})`);
        
        // 移动到下一个角点
        autoCalibState.currentPickIndex++;
        
        // 更新UI
        updatePickProgress();
        
        if (autoCalibState.currentPickIndex >= autoCalibState.corners.length) {
            addLog('success', '🎉 所有角点点选完成！');
            addLog('info', '💡 现在可以保存所有位姿到文件，或直接开始自动标定');
        }
        
    } catch (error) {
        addLog('error', `❌ 获取位姿失败：${error.message}`);
        console.error('记录点选位姿失败:', error);
    }
}

// 清除角点点选数据
function handleClearPickData() {
    if (autoCalibState.corners.length === 0 && autoCalibState.pickPoses.length === 0) {
        addLog('info', '📝 角点数据已经是空的');
        return;
    }
    
    autoCalibState.corners = [];
    autoCalibState.pickPoses = [];
    autoCalibState.pickImage = null;
    autoCalibState.currentPickIndex = 0;
    
    addLog('info', '🗑️ 已清除所有角点点选数据');
    
    // 更新UI
    updatePickProgress();
}

// 显示点选参考图像
function displayPickImage() {
    if (!autoCalibState.pickImage) return;
    
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
        imgElement.onerror = function() {
            addLog('error', '❌ 图像加载失败');
        };
        imgElement.src = autoCalibState.pickImage;
    }
}

// 更新点选进度显示
function updatePickProgress() {
    const progressDiv = document.getElementById('auto-pick-progress');
    const listDiv = document.getElementById('auto-pick-poses-list');
    
    if (!progressDiv) return;
    
    if (autoCalibState.corners.length === 0) {
        progressDiv.innerHTML = '<span style="color: #666;">尚未采集角点数据</span>';
        if (listDiv) listDiv.style.display = 'none';
        return;
    }
    
    const total = autoCalibState.corners.length;
    const recorded = autoCalibState.pickPoses.length;
    const progress = (recorded / total * 100).toFixed(0);
    
    progressDiv.innerHTML = `
        <div style="display: flex; justify-content: space-between; margin-bottom: 4px;">
            <span><strong>进度：</strong>${recorded}/${total} 个角点</span>
            <span><strong>${progress}%</strong></span>
        </div>
        <div style="width: 100%; height: 8px; background: #e0e0e0; border-radius: 4px; overflow: hidden;">
            <div style="width: ${progress}%; height: 100%; background: ${recorded === total ? '#4caf50' : '#ff9800'}; transition: width 0.3s;"></div>
        </div>
        ${recorded < total ? `<p style="margin: 4px 0 0 0; color: #f57c00; font-size: 10px;">💡 当前角点：#${autoCalibState.corners[autoCalibState.currentPickIndex].index}</p>` : '<p style="margin: 4px 0 0 0; color: #4caf50; font-size: 10px;">✅ 所有角点已完成</p>'}
    `;
    
    // 显示角点列表
    if (listDiv && autoCalibState.corners.length > 0) {
        listDiv.style.display = 'block';
        let html = '<div style="display: grid; grid-template-columns: repeat(10, 1fr); gap: 4px; padding: 8px; background: white; border-radius: 4px;">';
        
        autoCalibState.corners.forEach((corner, i) => {
            const status = i < recorded ? '✅' : (i === autoCalibState.currentPickIndex ? '📍' : '⭕');
            const color = i < recorded ? '#4caf50' : (i === autoCalibState.currentPickIndex ? '#ff9800' : '#999');
            html += `<div style="padding: 4px; text-align: center; background: #f5f5f5; border-radius: 3px; font-size: 10px; color: ${color};">${status}${corner.index}</div>`;
        });
        
        html += '</div>';
        listDiv.innerHTML = html;
    }
}

// 继续实现剩余功能...
// （由于字数限制，我会分多个文件继续）

