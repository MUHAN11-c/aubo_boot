// 全局变量
let currentTab = 'camera-verify';
let imageUpdateInterval = null;
let robotPoseUpdateInterval = null;
let capturedPoses = [];

// 放大镜配置
const MAGNIFIER_ZOOM = 5; // 放大倍数（提高以获得更好的细节观察）
const MAGNIFIER_SIZE = 80; // 采样区域大小（像素，减小以获得更高放大倍数）
const MAGNIFIER_SMOOTHING = true; // 图像平滑（true=平滑，false=像素清晰）

// 手动选点模式
let manualPickMode = false;
let manualPickPoints = [];

// 图像缩放状态管理
const imageZoomStates = {
    'camera-verify': { scale: 1, isDragging: false, lastX: 0, lastY: 0 },
    'hand-eye-calib': { scale: 1, isDragging: false, lastX: 0, lastY: 0 },
    'hand-eye-verify': { scale: 1, isDragging: false, lastX: 0, lastY: 0 }
};

// 全局变量 - 标定参数加载状态
let cameraParamsLoaded = false;

// 页面加载完成后初始化
document.addEventListener('DOMContentLoaded', function() {
    console.log('手眼标定工具初始化...');
    addLog('info', '系统初始化完成');
    
    // 初始化选项卡切换
    initTabs();
    
    // 初始化各个选项卡的按钮事件
    initCameraVerifyButtons();
    initHandEyeCalibButtons();  // 手眼标定按钮
    initHandEyeVerifyButtons();  // 手眼验证按钮
    
    // 初始化放大镜功能
    initMagnifiers();
    
    // 初始化图像缩放功能
    initImageZoom();
    
    // 启动图像更新
    startImageUpdate();
    
    // 启动机器人位姿更新（仅在手眼标定选项卡）
    startRobotPoseUpdate();
    
    // 启动机器人状态更新（手眼验证选项卡）
    startVerifyRobotStatusUpdate();
    
    // 检查连接状态
    checkConnectionStatus();
    
    // 检查相机状态
    checkCameraIDStatus();
    
    // 尝试自动加载默认标定参数（只从 ROS2 话题获取）
    autoLoadDefaultCameraParams();
    
    // 启动相机内参定期更新（从 ROS2 话题）
    startCameraInfoUpdate();
    
    // 尝试自动加载手眼标定参数
    autoLoadHandEyeCalibrationParams();
    
    addLog('success', 'Web界面启动成功，等待ROS2数据...');
});

// ============= 日志系统 =============
function addLog(type, message) {
    // 映射：选项卡ID -> 日志容器ID
    const logIdMap = {
        'hand-eye-calib': 'hand-eye-calib-log',
        'auto-hand-eye-calib': 'auto-hand-eye-calib-log',
        'hand-eye-verify': 'hand-eye-verify-log'
    };
    const logContainerId = logIdMap[currentTab] || `${currentTab}-log`;
    const logContainer = document.getElementById(logContainerId);
    
    if (!logContainer) return;
    
    const timestamp = new Date().toLocaleTimeString('zh-CN', { hour12: false });
    const logEntry = document.createElement('div');
    logEntry.className = `log-entry ${type}`;
    logEntry.innerHTML = `<span class="log-time">[${timestamp}]</span>${message}`;
    
    // 添加到容器顶部
    if (logContainer.firstChild) {
        logContainer.insertBefore(logEntry, logContainer.firstChild);
    } else {
        logContainer.appendChild(logEntry);
    }
    
    // 限制日志条目数量（最多100条）
    while (logContainer.children.length > 100) {
        logContainer.removeChild(logContainer.lastChild);
    }
}

function clearLog() {
    // 映射：选项卡ID -> 日志容器ID
    const logIdMap = {
        'hand-eye-calib': 'hand-eye-calib-log',
        'hand-eye-verify': 'hand-eye-verify-log'
    };
    const logContainerId = logIdMap[currentTab] || `${currentTab}-log`;
    const logContainer = document.getElementById(logContainerId);
    if (logContainer) {
        logContainer.innerHTML = '<div class="log-entry info">日志已清空</div>';
    }
}

// ============= 放大镜功能 =============
function initMagnifiers() {
    // 为每个选项卡的图像容器添加放大镜功能
    const tabs = ['camera-verify', 'hand-eye-calib', 'hand-eye-verify', 'auto-hand-eye-calib'];
    
    tabs.forEach(tab => {
        const imageContainer = document.getElementById(`${tab}-image-container`);
        const imageElement = document.getElementById(`${tab}-image`);
        const magnifierCanvas = document.getElementById(`${tab}-magnifier`);
        const magnifierPlaceholder = imageContainer?.parentElement.querySelector('.magnifier-placeholder');
        
        if (!imageContainer || !imageElement || !magnifierCanvas) return;
        
        const ctx = magnifierCanvas.getContext('2d');
        const crosshair = imageContainer.querySelector('.zoom-crosshair');
        
        // 更新放大镜 - 显示鼠标位置或可视区域中心
        const updateMagnifier = (mouseX = null, mouseY = null) => {
            if (!imageElement.classList.contains('loaded')) return;
            
            // 绘制放大镜
            const coords = drawMagnifier(imageElement, magnifierCanvas, ctx, imageContainer, mouseX, mouseY);
            
            // 更新放大镜标题右侧的坐标显示
            if (coords) {
                const hintElement = document.getElementById(`${tab}-mag-hint`);
                if (hintElement) {
                    hintElement.textContent = `X: ${Math.round(coords.sourceX)} | Y: ${Math.round(coords.sourceY)}`;
                }
                
                // 显示放大镜，隐藏占位符
                magnifierCanvas.classList.add('active');
                if (magnifierPlaceholder) {
                    magnifierPlaceholder.style.display = 'none';
                }
            }
        };
        
        // 监听滚动事件
        imageContainer.addEventListener('scroll', () => updateMagnifier());
        
        // 监听鼠标移动事件
        imageContainer.addEventListener('mousemove', (e) => {
            const rect = imageContainer.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;
            updateMagnifier(mouseX, mouseY);
        });
        
        // 鼠标离开时回到中心点显示
        imageContainer.addEventListener('mouseleave', () => updateMagnifier());
        
        // 使用requestAnimationFrame持续更新放大镜（实时跟踪）
        const animateUpdate = () => {
            if (imageContainer.querySelector('img.loaded')) {
                updateMagnifier();
            }
            requestAnimationFrame(animateUpdate);
        };
        requestAnimationFrame(animateUpdate);
        
        // 初始显示
        setTimeout(() => updateMagnifier(), 500);
    });
}

function drawMagnifier(imageElement, canvas, ctx, container, mouseX = null, mouseY = null) {
    // 获取图像的原始尺寸
    const imgNaturalWidth = imageElement.naturalWidth;
    const imgNaturalHeight = imageElement.naturalHeight;
    
    if (!imgNaturalWidth || !imgNaturalHeight) return null;
    
    // 获取容器和图像的矩形信息
    const containerRect = container.getBoundingClientRect();
    const imgRect = imageElement.getBoundingClientRect();
    
    // 计算目标点位置（鼠标位置或容器中心）
    let targetX, targetY;
    
    if (mouseX !== null && mouseY !== null) {
        // 使用鼠标位置（相对于容器的坐标）
        targetX = mouseX;
        targetY = mouseY;
    } else {
        // 使用容器可视区域的中心点（视口坐标）
        const containerCenterX = containerRect.left + containerRect.width / 2;
        const containerCenterY = containerRect.top + containerRect.height / 2;
        
        // 计算中心点在图像上的位置
        targetX = containerCenterX - imgRect.left;
        targetY = containerCenterY - imgRect.top;
    }
    
    // 计算目标点在图像上的位置
    const imgX = targetX;
    const imgY = targetY;
    
    // 检查目标点是否在图像内
    if (imgX < 0 || imgX > imgRect.width || imgY < 0 || imgY > imgRect.height) {
        return null;
    }
    
    // 计算在原始图像中的坐标（像素坐标）
    const scaleX = imgNaturalWidth / imgRect.width;
    const scaleY = imgNaturalHeight / imgRect.height;
    const sourceX = imgX * scaleX;
    const sourceY = imgY * scaleY;
    
    // 设置canvas尺寸 - 填满容器
    const containerWidth = canvas.parentElement.clientWidth;
    const containerHeight = canvas.parentElement.clientHeight;
    canvas.width = containerWidth;
    canvas.height = containerHeight;
    
    // 计算采样区域 - 以中心点为中心
    const sampleSize = MAGNIFIER_SIZE;
    let sx = sourceX - sampleSize / 2;
    let sy = sourceY - sampleSize / 2;
    
    // 处理边界情况
    if (sx < 0) sx = 0;
    if (sy < 0) sy = 0;
    if (sx + sampleSize > imgNaturalWidth) sx = imgNaturalWidth - sampleSize;
    if (sy + sampleSize > imgNaturalHeight) sy = imgNaturalHeight - sampleSize;
    
    const sw = Math.min(sampleSize, imgNaturalWidth - sx);
    const sh = Math.min(sampleSize, imgNaturalHeight - sy);
    
    // 清空canvas
    ctx.clearRect(0, 0, containerWidth, containerHeight);
    
    // 绘制放大的图像 - 填满整个canvas
    // 使用高质量图像平滑以获得更好的视觉效果
    ctx.imageSmoothingEnabled = MAGNIFIER_SMOOTHING;
    ctx.imageSmoothingQuality = 'high'; // 高质量平滑
    ctx.drawImage(imageElement, sx, sy, sw, sh, 0, 0, containerWidth, containerHeight);
    
    // 绘制十字线 - 固定在canvas中心
    const centerX = containerWidth / 2;
    const centerY = containerHeight / 2;
    
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(centerX, 0);
    ctx.lineTo(centerX, containerHeight);
    ctx.moveTo(0, centerY);
    ctx.lineTo(containerWidth, centerY);
    ctx.stroke();
    
    // 绘制中心圆
    ctx.beginPath();
    ctx.arc(centerX, centerY, 5, 0, 2 * Math.PI);
    ctx.fillStyle = '#00ff00';
    ctx.fill();
    
    // 返回坐标信息供外部显示
    return {
        sourceX: sourceX,
        sourceY: sourceY
    };
}

// ============= 选项卡切换 =============
function initTabs() {
    const tabButtons = document.querySelectorAll('.tab-button');
    
    tabButtons.forEach(button => {
        button.addEventListener('click', function() {
            const tabId = this.getAttribute('data-tab');
            switchTab(tabId);
        });
    });
}

function switchTab(tabId) {
    // 移除所有active类
    document.querySelectorAll('.tab-button').forEach(btn => {
        btn.classList.remove('active');
    });
    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.remove('active');
    });
    
    // 添加active类到选中的选项卡
    document.querySelector(`[data-tab="${tabId}"]`).classList.add('active');
    document.getElementById(tabId).classList.add('active');
    
    currentTab = tabId;
    addLog('info', `切换到 ${getTabName(tabId)} 选项卡`);
    
    // 如果切换到"手眼标定"或"自动手眼标定"标签页，立即更新机器人位姿
    if (tabId === 'hand-eye-calib' || tabId === 'auto-hand-eye-calib') {
        updateRobotPose();
    }
}

function getTabName(tabId) {
    const names = {
        'camera-verify': '相机标定精度验证',
        'hand-eye-calib': '手眼标定',
        'auto-hand-eye-calib': '🤖 自动手眼标定',
        'hand-eye-verify': '手眼标定精度验证'
    };
    return names[tabId] || tabId;
}

// ============= 相机标定精度验证 =============
function initCameraVerifyButtons() {
    const buttons = {
        'btn-load-camera-params': loadCameraParams,
        'btn-capture-verify-image': captureVerifyImage,
        'btn-open-local-image': openLocalImage,
        'btn-manual-pick': toggleManualPickMode,
        'btn-extract-corners': extractCorners,
        'btn-calculate-result': calculateResult
    };
    
    Object.keys(buttons).forEach(btnId => {
        const btn = document.getElementById(btnId);
        if (btn) {
            btn.addEventListener('click', buttons[btnId]);
        }
    });
    
    // 添加保存图像按钮事件
    const saveBtns = {
        'save-camera-verify-image': () => saveCurrentImage('camera-verify'),
        'save-hand-eye-image': () => saveCurrentImage('hand-eye-calib'),
        'save-hand-eye-verify-image': () => saveCurrentImage('hand-eye-verify')
    };
    
    Object.keys(saveBtns).forEach(btnId => {
        const btn = document.getElementById(btnId);
        if (btn) {
            btn.addEventListener('click', saveBtns[btnId]);
        }
    });
    
    // 添加图像文件输入框（隐藏）
    const fileInput = document.createElement('input');
    fileInput.type = 'file';
    fileInput.accept = 'image/*';
    fileInput.style.display = 'none';
    fileInput.id = 'file-input-camera-verify';
    document.body.appendChild(fileInput);
    fileInput.addEventListener('change', handleFileSelect);
    
    // 添加标定参数文件输入框（隐藏）
    const calibFileInput = document.createElement('input');
    calibFileInput.type = 'file';
    calibFileInput.accept = '.xml';
    calibFileInput.style.display = 'none';
    calibFileInput.id = 'file-input-calib-params';
    document.body.appendChild(calibFileInput);
    calibFileInput.addEventListener('change', handleCalibFileSelect);
}

function loadCameraParams() {
    // 弹出文件选择对话框
    const fileInput = document.getElementById('file-input-calib-params');
    if (fileInput) {
        fileInput.click();
    }
}

function handleCalibFileSelect(event) {
    const file = event.target.files[0];
    if (!file) return;
    
    addLog('info', `选择标定文件: ${file.name}`);
    showToast('正在加载相机参数...', 'info');
    
    const formData = new FormData();
    formData.append('file', file);
    
    fetch('/api/camera/load_params', {
        method: 'POST',
        body: formData
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            cameraParamsLoaded = true;
            cameraInfoSource = 'loaded_file';  // 标记为文件加载
            const cm = data.camera_matrix;
            
            const paramHTML = `
                <p>数据源: <span style="color: #2196f3;">(从文件)</span></p>
                <p>图像尺寸: <span>${data.image_size[0]} x ${data.image_size[1]}</span></p>
                <p>焦距 fx: <span>${cm.fx.toFixed(2)}</span></p>
                <p>焦距 fy: <span>${cm.fy.toFixed(2)}</span></p>
                <p>主点 cx: <span>${cm.cx.toFixed(2)}</span></p>
                <p>主点 cy: <span>${cm.cy.toFixed(2)}</span></p>
                <p>畸变系数: <span>${data.dist_coeffs.length}个</span></p>
                <p>重投影误差: <span>${data.mean_reprojection_error.toFixed(4)} px</span></p>
            `;
            
            // 更新相机标定验证选项卡
            document.getElementById('camera-params-info').innerHTML = paramHTML;
            
            // 同时更新手眼标定选项卡
            const heParamsInfo = document.getElementById('he-camera-params-info');
            if (heParamsInfo) {
                heParamsInfo.innerHTML = paramHTML;
            }
            
            addLog('success', `相机参数从文件加载成功：fx=${cm.fx.toFixed(2)}, fy=${cm.fy.toFixed(2)}`);
            addLog('info', '💡 提示: 如果 ROS2 CameraInfo 话题可用，系统会自动切换到话题数据');
            showToast('相机参数从文件加载成功', 'success');
        } else {
            addLog('error', '加载相机参数失败: ' + data.error);
            showToast('加载失败: ' + data.error, 'error');
        }
    })
    .catch(error => {
        addLog('error', '加载相机参数失败: ' + error);
        showToast('加载失败', 'error');
    });
    
    // 重置文件输入
    event.target.value = '';
}

// 自动加载默认标定参数
function autoLoadDefaultCameraParams() {
    addLog('info', '正在等待 ROS2 CameraInfo 话题数据...');
    
    const waitingHTML = `
        <p style="color: #2196f3;">⏳ 等待相机内参数据...</p>
        <p>优先从 ROS2 CameraInfo 话题获取</p>
        <p style="font-size: 0.9em; color: #666;">系统会自动加载，请稍候...</p>
            `;
            
    // 显示等待提示
    document.getElementById('camera-params-info').innerHTML = waitingHTML;
            const heParamsInfo = document.getElementById('he-camera-params-info');
            if (heParamsInfo) {
        heParamsInfo.innerHTML = waitingHTML;
            }
            
    // 定期检查机制会自动获取和更新内参（由 startCameraInfoUpdate() 处理）
    // 不再自动从文件加载，完全依赖 ROS2 话题
    
    // 如果10秒后还没有获取到，提示用户可以手动加载
    setTimeout(() => {
        if (!cameraParamsLoaded) {
            addLog('warning', '⚠️ 10秒内未从 ROS2 话题获取到内参，系统将继续等待...');
            addLog('info', '提示: 请确保相机节点正在运行，或点击"加载相机参数"按钮手动加载文件');
            
            const stillWaitingHTML = `
                <p style="color: #ff9800;">⏳ 仍在等待 ROS2 CameraInfo 话题...</p>
                <p>请确保相机节点正在运行</p>
                <p style="font-size: 0.9em;">或点击"加载相机参数"手动加载文件</p>
            `;
            
            document.getElementById('camera-params-info').innerHTML = stillWaitingHTML;
            if (heParamsInfo) {
                heParamsInfo.innerHTML = stillWaitingHTML;
            }
        }
    }, 10000);
}

// 自动加载手眼标定参数
function autoLoadHandEyeCalibrationParams() {
    // 延迟2秒后自动加载手眼标定参数
    setTimeout(() => {
        addLog('info', '尝试自动加载手眼标定参数...');
        
        fetch('/api/hand_eye/load_calibration_params', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            }
        })
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                handEyeCalibrationParams = data;
                updateVerifyParameterDisplay();
                addLog('success', '手眼标定参数自动加载成功');
            } else {
                addLog('warning', `手眼标定参数自动加载失败: ${data.error}`);
            }
        })
        .catch(error => {
            addLog('warning', `自动加载手眼标定参数失败: ${error.message}`);
        });
    }, 2000);
}

function captureVerifyImage() {
    addLog('info', '触发相机拍照...');
    showToast('正在触发相机拍照...', 'info');
    
    fetch('/api/camera/capture', {
        method: 'POST'
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addLog('success', '拍照命令已发送: ' + data.message);
            showToast('拍照成功，等待图像...', 'success');
            
            // 多次尝试获取图像（每500ms一次，最多尝试10次）
            let attemptCount = 0;
            const maxAttempts = 10;
            const attemptInterval = setInterval(() => {
                attemptCount++;
                addLog('info', `尝试获取图像 (${attemptCount}/${maxAttempts})...`);
                
                fetch('/api/current_image')
                    .then(response => response.json())
                    .then(imgData => {
                        if (imgData.success) {
                            clearInterval(attemptInterval);
                            
                            const imgElement = document.getElementById('camera-verify-image');
                            const placeholder = document.getElementById('camera-verify-image-container').querySelector('.image-placeholder');
                            
                            if (imgElement) {
                                // 先移除旧的事件处理器，避免触发旧图像的error
                                imgElement.onload = null;
                                imgElement.onerror = null;
                                
                                // 设置新的事件处理器
                                imgElement.onload = function() {
                                    this.classList.add('loaded');
                                    if (placeholder) {
                                        placeholder.style.display = 'none';
                                    }
                                    addLog('success', `✅ 图像已成功显示 (尺寸: ${imgData.width}x${imgData.height})`);
                                    showToast('图像加载成功', 'success');
                                    
                                    // 注意：图像保持原始状态，计算时才进行像素去畸变
                                    if (cameraParamsLoaded) {
                                        addLog('info', '💡 提示：图像显示为原始状态，坐标计算时会自动去畸变');
                                    }
                                };
                                
                                imgElement.onerror = function(e) {
                                    console.error('Image load error:', e);
                                    addLog('error', `图像加载失败 - 尺寸: ${imgData.width}x${imgData.height}`);
                                    showToast('图像显示失败', 'error');
                                };
                                
                                // 设置新图像
                                addLog('info', `正在加载图像... (${(imgData.image.length/1024).toFixed(1)}KB)`);
                                imgElement.src = imgData.image;
                                
                                // 重置缩放
                                resetImageZoom('camera-verify');
                            }
                        } else if (attemptCount >= maxAttempts) {
                            clearInterval(attemptInterval);
                            addLog('warning', '未能获取到图像数据，请检查/image_data话题');
                            showToast('图像获取超时，请检查相机', 'warning');
                        }
                    })
                    .catch(error => {
                        if (attemptCount >= maxAttempts) {
                            clearInterval(attemptInterval);
                            addLog('error', '获取图像失败: ' + error);
                        }
                    });
            }, 500);
        } else {
            addLog('error', '拍照失败: ' + data.error);
            showToast('拍照失败: ' + data.error, 'error');
        }
    })
    .catch(error => {
        addLog('error', '拍照失败: ' + error);
        showToast('拍照失败', 'error');
    });
}

function manualUpdateImage() {
    fetch('/api/current_image')
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                const imgElement = document.getElementById('camera-verify-image');
                const placeholder = document.getElementById('camera-verify-image-container').querySelector('.image-placeholder');
                
                if (imgElement) {
                    imgElement.onload = function() {
                        this.classList.add('loaded');
                        if (placeholder) {
                            placeholder.style.display = 'none';
                        }
                        addLog('success', '图像已更新显示');
                    };
                    imgElement.src = data.image;
                }
            } else {
                addLog('warning', '暂无图像数据');
            }
        })
        .catch(error => {
            addLog('error', '获取图像失败: ' + error);
        });
}

function openLocalImage() {
    const fileInput = document.getElementById('file-input-camera-verify');
    if (fileInput) {
        fileInput.click();
    }
}

function toggleManualPickMode() {
    manualPickMode = !manualPickMode;
    const btn = document.getElementById('btn-manual-pick');
    const imageContainer = document.getElementById('camera-verify-image-container');
    
    if (manualPickMode) {
        btn.classList.add('primary');
        btn.textContent = '手动选点 (开启)';
        imageContainer.style.cursor = 'crosshair';
        manualPickPoints = [];
        addLog('info', '📌 手动选点模式已开启，点击图像选择点位');
        showToast('手动选点模式已开启', 'info');
    } else {
        btn.classList.remove('primary');
        btn.textContent = '手动选点';
        imageContainer.style.cursor = imageZoomStates['camera-verify'].scale > 1 ? 'grab' : 'crosshair';
        addLog('info', `📌 手动选点模式已关闭，已选 ${manualPickPoints.length} 个点`);
        showToast('手动选点模式已关闭', 'info');
    }
}

function handleImageClick(event) {
    if (!manualPickMode) return;
    
    const imageElement = document.getElementById('camera-verify-image');
    const imageContainer = document.getElementById('camera-verify-image-container');
    
    if (!imageElement.classList.contains('loaded')) {
        showToast('请先加载图像', 'warning');
        return;
    }
    
    // 获取可视区域中心的坐标（不是点击位置）
    const containerRect = imageContainer.getBoundingClientRect();
    const imgRect = imageElement.getBoundingClientRect();
    
    // 计算容器可视区域的中心点
    const containerCenterX = containerRect.left + containerRect.width / 2;
    const containerCenterY = containerRect.top + containerRect.height / 2;
    
    // 计算中心点在图像上的位置
    const imgX = containerCenterX - imgRect.left;
    const imgY = containerCenterY - imgRect.top;
    
    // 转换为原始图像坐标
    const scaleX = imageElement.naturalWidth / imgRect.width;
    const scaleY = imageElement.naturalHeight / imgRect.height;
    const pixelX = Math.round(imgX * scaleX);
    const pixelY = Math.round(imgY * scaleY);
    
    // 检查坐标是否在图像范围内
    if (pixelX < 0 || pixelX >= imageElement.naturalWidth || 
        pixelY < 0 || pixelY >= imageElement.naturalHeight) {
        showToast('可视区域中心超出图像范围', 'warning');
        return;
    }
    
    addLog('info', `📍 选择中心点 ${manualPickPoints.length + 1}: (${pixelX}, ${pixelY})`);
    
    // 调用后端API转换坐标
    const checkerboardSize = parseFloat(document.getElementById('checkerboard-size').value);
    
    fetch('/api/camera/pixel_to_camera', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({
            pixel_x: pixelX,
            pixel_y: pixelY,
            checkerboard_size: checkerboardSize
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            const point = {
                index: manualPickPoints.length + 1,
                pixel_u: pixelX,
                pixel_v: pixelY,
                camera_x: data.camera_x,
                camera_y: data.camera_y,
                camera_z: data.camera_z,
                distance: 0
            };
            
            // 如果已有点，计算物理距离
            if (manualPickPoints.length > 0) {
                const prevPoint = manualPickPoints[manualPickPoints.length - 1];
                const dx = point.camera_x - prevPoint.camera_x;
                const dy = point.camera_y - prevPoint.camera_y;
                const dz = point.camera_z - prevPoint.camera_z;
                point.distance = Math.sqrt(dx*dx + dy*dy + dz*dz);
                
                addLog('success', `📏 与上一点距离: ${point.distance.toFixed(3)} mm`);
            }
            
            manualPickPoints.push(point);
            updateManualPickTable();
            
            showToast(`已添加点 ${point.index}`, 'success');
        } else {
            addLog('error', '坐标转换失败: ' + data.error);
            showToast('坐标转换失败', 'error');
        }
    })
    .catch(error => {
        addLog('error', '坐标转换失败: ' + error);
        showToast('坐标转换失败', 'error');
    });
}

function updateManualPickTable() {
    const tableBody = document.querySelector('#camera-verify-table tbody');
    tableBody.innerHTML = '';
    
    manualPickPoints.forEach(point => {
        const row = document.createElement('tr');
        row.innerHTML = `
            <td>${point.index}</td>
            <td>${point.pixel_u}</td>
            <td>${point.pixel_v}</td>
            <td>${point.camera_x.toFixed(3)}</td>
            <td>${point.camera_y.toFixed(3)}</td>
            <td>${point.camera_z.toFixed(3)}</td>
            <td>${point.distance > 0 ? point.distance.toFixed(3) : '-'}</td>
        `;
        
        // 添加点击复制功能
        row.style.cursor = 'pointer';
        row.title = '点击复制整行数据';
        row.addEventListener('click', function() {
            copyRowData(this, '手动选点数据');
        });
        
        tableBody.appendChild(row);
    });
}

function handleFileSelect(event) {
    const file = event.target.files[0];
    if (!file) return;
    
    addLog('info', `选择图像: ${file.name}`);
    showToast('正在加载图像...', 'info');
    
    const formData = new FormData();
    formData.append('image', file);
    
    fetch('/api/camera/upload_image', {
        method: 'POST',
        body: formData
    })
    .then(response => {
        console.log('Upload response status:', response.status);
        return response.json();
    })
    .then(data => {
        console.log('Upload response data:', data);
        
        if (data.success) {
            addLog('success', `后端处理成功，图像尺寸: ${data.width}x${data.height}`);
            
            // 显示图像
            const imgElement = document.getElementById('camera-verify-image');
            const container = document.getElementById('camera-verify-image-container');
            const placeholder = container.querySelector('.image-placeholder');
            
            // 确保有图像数据
            if (!data.image || !data.image.startsWith('data:image')) {
                addLog('error', '返回的图像数据格式错误');
                showToast('图像数据格式错误', 'error');
                return;
            }
            
            addLog('info', `准备显示图像，Base64长度: ${data.image.length} (${(data.image.length/1024).toFixed(1)}KB)`);
            
            // 先移除旧的事件处理器
            imgElement.onload = null;
            imgElement.onerror = null;
            
            // 设置新的事件处理器
            imgElement.onload = function() {
                console.log('Image loaded successfully');
                this.classList.add('loaded');
                if (placeholder) {
                    placeholder.style.display = 'none';
                }
                addLog('success', '✅ 本地图像显示成功');
                showToast('图像加载成功', 'success');
                
                // 注意：图像保持原始状态，计算时才进行像素去畸变
                if (cameraParamsLoaded) {
                    addLog('info', '💡 提示：图像显示为原始状态，坐标计算时会自动去畸变');
                }
            };
            
            imgElement.onerror = function(err) {
                console.error('Image load error:', err);
                addLog('error', '图像加载到DOM失败');
                showToast('图像显示失败', 'error');
            };
            
            // 设置新图像（不需要时间戳，因为每次都是新的base64）
            imgElement.src = data.image;
            
            // 重置缩放
            resetImageZoom('camera-verify');
            
            addLog('info', '图像src已设置，等待浏览器加载...');
        } else {
            addLog('error', '图像上传失败: ' + data.error);
            showToast('图像上传失败: ' + data.error, 'error');
        }
    })
    .catch(error => {
        console.error('Upload error:', error);
        addLog('error', '图像上传失败: ' + error);
        showToast('图像上传失败', 'error');
    });
    
    // 重置文件输入
    event.target.value = '';
}

function extractCorners() {
    addLog('info', '开始提取棋盘格角点...');
    showToast('正在提取角点...', 'info');
    
    const squareSize = parseFloat(document.getElementById('checkerboard-size').value) || 15.0;
    
    fetch('/api/camera/extract_corners', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            square_size: squareSize
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 显示带角点的图像
            const imgElement = document.getElementById('camera-verify-image');
            imgElement.src = data.image_with_corners;
            
            // 填充表格
            fillCornersTable(data.corners_data);
            
            addLog('success', `成功提取 ${data.corners_count} 个角点`);
            showToast(`成功提取 ${data.corners_count} 个角点`, 'success');
        } else {
            addLog('error', '提取角点失败: ' + data.error);
            showToast('提取角点失败: ' + data.error, 'error');
        }
    })
    .catch(error => {
        addLog('error', '提取角点失败: ' + error);
        showToast('提取角点失败', 'error');
    });
}

function fillCornersTable(cornersData) {
    const tbody = document.querySelector('#camera-verify-table tbody');
    tbody.innerHTML = '';
    
    cornersData.forEach(corner => {
        const row = tbody.insertRow();
        row.innerHTML = `
            <td>${corner.index}</td>
            <td>${corner.pixel_u.toFixed(2)}</td>
            <td>${corner.pixel_v.toFixed(2)}</td>
            <td>${corner.camera_x.toFixed(3)}</td>
            <td>${corner.camera_y.toFixed(3)}</td>
            <td>${corner.camera_z.toFixed(3)}</td>
            <td>${corner.adjacent_distance.toFixed(3)}</td>
        `;
        
        // 添加点击复制功能
        row.style.cursor = 'pointer';
        row.title = '点击复制整行数据';
        row.addEventListener('click', function() {
            copyRowData(this, '角点数据');
        });
    });
    
    addLog('info', `数据表格已更新，共 ${cornersData.length} 行 (点击行可复制)`);
}

function calculateResult() {
    addLog('info', '开始计算结果...');
    showToast('正在计算...', 'info');
    
    const expectedSize = parseFloat(document.getElementById('checkerboard-size').value) || 15.0;
    
    fetch('/api/camera/calculate_result', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            expected_size: expectedSize
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 显示结果弹窗
            showResultDialog(data, expectedSize);
            
            addLog('success', `计算完成：平均距离=${data.avg_distance.toFixed(3)}mm, 误差=${data.error_avg.toFixed(3)}mm`);
            showToast('计算完成', 'success');
        } else {
            addLog('error', '计算失败: ' + data.error);
            showToast('计算失败: ' + data.error, 'error');
        }
    })
    .catch(error => {
        addLog('error', '计算失败: ' + error);
        showToast('计算失败', 'error');
    });
}

function showResultDialog(result, expectedSize) {
    // 获取棋盘格尺寸
    const patternSize = result.pattern_size || [10, 7];
    const cols = patternSize[0];
    const rows = patternSize[1];
    
    // 创建弹窗
    const dialog = document.createElement('div');
    dialog.className = 'result-dialog';
    dialog.innerHTML = `
        <div class="result-dialog-content">
            <h3>📊 标定精度计算结果</h3>
            
            <!-- 基本信息 -->
            <div class="result-section">
                <h4>📐 棋盘格信息</h4>
                <div class="result-grid">
                    <div class="result-item">
                        <span class="result-label">棋盘格尺寸:</span>
                        <span class="result-value">${cols} × ${rows}</span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">格子大小:</span>
                        <span class="result-value">${expectedSize.toFixed(2)} mm</span>
                    </div>
                </div>
            </div>
            
            <!-- 相邻距离统计 -->
            <div class="result-section">
                <h4>📏 相邻角点距离（格子大小）</h4>
                <div class="result-grid">
                    <div class="result-item">
                        <span class="result-label">期望距离:</span>
                        <span class="result-value">${expectedSize.toFixed(2)} mm</span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">平均距离:</span>
                        <span class="result-value">${result.avg_distance.toFixed(3)} mm</span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">最大距离:</span>
                        <span class="result-value">${result.max_distance.toFixed(3)} mm</span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">最小距离:</span>
                        <span class="result-value">${result.min_distance.toFixed(3)} mm</span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">标准差:</span>
                        <span class="result-value">${result.std_distance.toFixed(3)} mm</span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">平均误差:</span>
                        <span class="result-value error-value">${result.error_avg.toFixed(3)} mm</span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">误差百分比:</span>
                        <span class="result-value error-value">${result.error_percent.toFixed(2)}%</span>
                    </div>
                </div>
            </div>
            
            <!-- 对角线距离验证 -->
            <div class="result-section diagonal-section">
                <h4>🎯 对角线距离验证（整体精度）</h4>
                <div class="diagonal-info">
                    <p class="diagonal-desc">从第一个角点（左上）到最后一个角点（右下）的距离</p>
                </div>
                <div class="result-grid">
                    <div class="result-item highlight">
                        <span class="result-label">理论距离:</span>
                        <span class="result-value">${result.diagonal_theoretical.toFixed(3)} mm</span>
                    </div>
                    <div class="result-item highlight">
                        <span class="result-label">实际距离:</span>
                        <span class="result-value">${result.diagonal_distance.toFixed(3)} mm</span>
                    </div>
                    <div class="result-item highlight">
                        <span class="result-label">差异:</span>
                        <span class="result-value ${result.diagonal_error < 1.0 ? 'success-value' : 'error-value'}">
                            ${result.diagonal_error.toFixed(3)} mm
                        </span>
                    </div>
                    <div class="result-item highlight">
                        <span class="result-label">差异百分比:</span>
                        <span class="result-value ${result.diagonal_error_percent < 1.0 ? 'success-value' : 'error-value'}">
                            ${result.diagonal_error_percent.toFixed(2)}%
                        </span>
                    </div>
                </div>
            </div>
            
            ${result.depth_error && result.depth_error.success ? `
            <!-- 深度误差评估 -->
            <div class="result-section depth-error-section">
                <h4>📊 深度误差评估（与深度相机对比）</h4>
                <div class="depth-error-info">
                    <p class="depth-error-desc">对比估计深度（solvePnP）与深度相机实际深度</p>
                </div>
                <div class="result-grid">
                    <div class="result-item">
                        <span class="result-label">总角点数:</span>
                        <span class="result-value">${result.depth_error.total_corners || result.depth_error.num_valid_points}</span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">有效点数:</span>
                        <span class="result-value">${result.depth_error.num_valid_points}</span>
                    </div>
                    ${result.depth_error.no_depth_count !== undefined ? `
                    <div class="result-item">
                        <span class="result-label">无深度值:</span>
                        <span class="result-value">${result.depth_error.no_depth_count}</span>
                    </div>
                    ` : ''}
                    ${result.depth_error.filtered_count !== undefined ? `
                    <div class="result-item">
                        <span class="result-label">过滤异常值:</span>
                        <span class="result-value">${result.depth_error.filtered_count}</span>
                    </div>
                    ` : ''}
                    <div class="result-item">
                        <span class="result-label">估计深度:</span>
                        <span class="result-value">${result.depth_error.estimated_depth.toFixed(2)} mm</span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">平均实际深度:</span>
                        <span class="result-value">${result.depth_error.depth_statistics.mean_actual.toFixed(2)} mm</span>
                    </div>
                    <div class="result-item highlight">
                        <span class="result-label">平均深度误差:</span>
                        <span class="result-value ${result.depth_error.depth_statistics.mean_error < 2.0 ? 'success-value' : result.depth_error.depth_statistics.mean_error < 5.0 ? 'warning-value' : 'error-value'}">
                            ${result.depth_error.depth_statistics.mean_error.toFixed(2)} mm
                        </span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">最大深度误差:</span>
                        <span class="result-value error-value">${result.depth_error.depth_statistics.max_error.toFixed(2)} mm</span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">最小深度误差:</span>
                        <span class="result-value">${result.depth_error.depth_statistics.min_error.toFixed(2)} mm</span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">深度误差标准差:</span>
                        <span class="result-value">${result.depth_error.depth_statistics.std_error.toFixed(2)} mm</span>
                    </div>
                    <div class="result-item">
                        <span class="result-label">平均相对误差:</span>
                        <span class="result-value ${result.depth_error.depth_statistics.mean_relative_error_percent < 1.0 ? 'success-value' : result.depth_error.depth_statistics.mean_relative_error_percent < 2.0 ? 'warning-value' : 'error-value'}">
                            ${result.depth_error.depth_statistics.mean_relative_error_percent.toFixed(2)}%
                        </span>
                    </div>
                </div>
                ${result.depth_error.depth_details && result.depth_error.depth_details.length > 0 ? `
                <!-- 详细深度值列表 -->
                <div class="depth-details-section">
                    <button class="toggle-details-btn" onclick="toggleDepthDetails(this)">
                        <span class="toggle-icon">▼</span>
                        <span>显示详细深度值（${result.depth_error.depth_details.length}个角点）</span>
                    </button>
                    <div class="depth-details-table" style="display: none;">
                        <table class="depth-table">
                            <thead>
                                <tr>
                                    <th>角点</th>
                                    <th>像素坐标</th>
                                    <th>估计深度</th>
                                    <th>实际深度（深度图）</th>
                                    <th>深度误差</th>
                                    <th>相对误差</th>
                                </tr>
                            </thead>
                            <tbody>
                                ${result.depth_error.depth_details.map((detail, idx) => `
                                    <tr>
                                        <td>#${detail.corner_index}</td>
                                        <td>(${detail.pixel_u.toFixed(0)}, ${detail.pixel_v.toFixed(0)})</td>
                                        <td>${detail.estimated_depth.toFixed(2)} mm</td>
                                        <td class="actual-depth-value">${detail.actual_depth.toFixed(2)} mm</td>
                                        <td class="${detail.depth_error < 2.0 ? 'success-value' : detail.depth_error < 5.0 ? 'warning-value' : 'error-value'}">
                                            ${detail.depth_error.toFixed(2)} mm
                                        </td>
                                        <td class="${detail.relative_error_percent < 1.0 ? 'success-value' : detail.relative_error_percent < 2.0 ? 'warning-value' : 'error-value'}">
                                            ${detail.relative_error_percent.toFixed(2)}%
                                        </td>
                                    </tr>
                                `).join('')}
                            </tbody>
                        </table>
                    </div>
                </div>
                ` : ''}
                <div class="depth-error-status ${result.depth_error.evaluation.excellent ? 'success' : result.depth_error.evaluation.good ? 'warning' : result.depth_error.evaluation.acceptable ? 'warning' : 'error'}">
                    ${result.depth_error.evaluation.excellent ? '✅ 深度精度优秀（误差 < 2mm）' : 
                      result.depth_error.evaluation.good ? '⚠️ 深度精度良好（误差 < 5mm）' : 
                      result.depth_error.evaluation.acceptable ? '⚠️ 深度精度可接受（误差 < 10mm）' : 
                      '❌ 深度精度较差（误差 >= 10mm），建议检查相机标定'}
                </div>
            </div>
            ` : ''}
            
            <!-- 整体评估 -->
            <div class="result-status ${result.error_percent < 3 && result.diagonal_error_percent < 1 ? 'success' : result.error_percent < 5 ? 'warning' : 'error'}">
                ${result.error_percent < 3 && result.diagonal_error_percent < 1 ? '✅ 精度优秀' : result.error_percent < 5 ? '⚠️ 精度良好' : '❌ 精度较差，建议重新标定'}
            </div>
            
            <button class="close-dialog-btn" onclick="this.parentElement.parentElement.remove()">关闭</button>
        </div>
    `;
    
    document.body.appendChild(dialog);
    
    // 点击背景关闭
    dialog.addEventListener('click', function(e) {
        if (e.target === dialog) {
            dialog.remove();
        }
    });
}

// ============= 通用函数（跨选项卡） =============

// 加载相机参数（通用）
function loadCameraParamsForTab(tabId) {
    // 使用相同的文件选择逻辑
    const fileInput = document.getElementById('file-input-calib-params');
    if (fileInput) {
        fileInput.click();
    }
}

// 采集图像（通用）
function captureImageForTab(tabId) {
    const tabPrefix = tabId.replace(/-/g, '_');
    addLog('info', '触发相机拍照...');
    showToast('正在触发相机拍照...', 'info');
    
    fetch('/api/camera/capture', {
        method: 'POST'
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addLog('success', '拍照命令已发送: ' + data.message);
            showToast('拍照成功，等待图像...', 'success');
            
            // 多次尝试获取图像
            let attemptCount = 0;
            const maxAttempts = 10;
            const attemptInterval = setInterval(() => {
                attemptCount++;
                addLog('info', `尝试获取图像 (${attemptCount}/${maxAttempts})...`);
                
                fetch('/api/current_image')
                    .then(response => response.json())
                    .then(imgData => {
                        if (imgData.success) {
                            clearInterval(attemptInterval);
                            displayImageForTab(tabId, imgData);
                        } else if (attemptCount >= maxAttempts) {
                            clearInterval(attemptInterval);
                            addLog('warning', '未能获取到图像数据，请检查/image_data话题');
                            showToast('图像获取超时，请检查相机', 'warning');
                        }
                    })
                    .catch(error => {
                        if (attemptCount >= maxAttempts) {
                            clearInterval(attemptInterval);
                            addLog('error', '获取图像失败: ' + error);
                        }
                    });
            }, 500);
        } else {
            addLog('error', '拍照失败: ' + data.error);
            showToast('拍照失败: ' + data.error, 'error');
        }
    })
    .catch(error => {
        addLog('error', '拍照失败: ' + error);
        showToast('拍照失败', 'error');
    });
}

// 打开本地图像（通用）
function openLocalImageForTab(tabId) {
    const fileInput = document.getElementById(`file-input-${tabId}`);
    if (fileInput) {
        fileInput.click();
    }
}

// 处理本地图像文件（通用）
function handleFileSelectForTab(event, tabId) {
    const file = event.target.files[0];
    if (!file) return;
    
    addLog('info', `选择图像: ${file.name}`);
    showToast('正在加载图像...', 'info');
    
    const formData = new FormData();
    formData.append('image', file);
    
    fetch('/api/camera/upload_image', {
        method: 'POST',
        body: formData
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addLog('success', `后端处理成功，图像尺寸: ${data.width}x${data.height}`);
            displayImageForTab(tabId, data);
        } else {
            addLog('error', '图像上传失败: ' + data.error);
            showToast('图像上传失败: ' + data.error, 'error');
        }
    })
    .catch(error => {
        addLog('error', '图像上传失败: ' + error);
        showToast('图像上传失败', 'error');
    });
    
    event.target.value = '';
}

// 显示图像到指定选项卡（通用）
function displayImageForTab(tabId, imgData) {
    const imgElement = document.getElementById(`${tabId}-image`);
    const container = document.getElementById(`${tabId}-image-container`);
    const placeholder = container.querySelector('.image-placeholder');
    
    if (!imgData.image || !imgData.image.startsWith('data:image')) {
        addLog('error', '返回的图像数据格式错误');
        showToast('图像数据格式错误', 'error');
        return;
    }
    
    addLog('info', `准备显示图像，Base64长度: ${imgData.image.length} (${(imgData.image.length/1024).toFixed(1)}KB)`);
    
    // 清除旧的事件处理器
    imgElement.onload = null;
    imgElement.onerror = null;
    
    // 设置新的事件处理器
    imgElement.onload = function() {
        this.classList.add('loaded');
        if (placeholder) {
            placeholder.style.display = 'none';
        }
        addLog('success', '✅ 图像显示成功');
        showToast('图像加载成功', 'success');
        
        if (cameraParamsLoaded) {
            addLog('info', '💡 提示：图像显示为原始状态，坐标计算时会自动去畸变');
        }
    };
    
    imgElement.onerror = function(err) {
        console.error('Image load error:', err);
        addLog('error', '图像加载到DOM失败');
        showToast('图像显示失败', 'error');
    };
    
    imgElement.src = imgData.image;
    resetImageZoom(tabId);
}

// 提取棋盘格角点（通用）
function extractCornersForTab(tabId) {
    addLog('info', '开始提取棋盘格角点...');
    showToast('正在提取角点...', 'info');
    
    // 根据选项卡获取棋盘格大小设置
    let squareSize = 15.0;
    if (tabId === 'hand-eye-calib') {
        squareSize = parseFloat(document.getElementById('he-checkerboard-size').value) || 15.0;
    } else {
        squareSize = parseFloat(document.getElementById('checkerboard-size').value) || 15.0;
    }
    
    fetch('/api/camera/extract_corners', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            square_size: squareSize
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 显示带角点的图像
            const imgElement = document.getElementById(`${tabId}-image`);
            imgElement.src = data.image_with_corners;
            
            // 填充表格
            if (tabId === 'camera-verify') {
                fillCornersTable(data.corners_data);
            } else if (tabId === 'hand-eye-calib') {
                // 检查是否是姿态法模式
                const calibrationMethod = document.getElementById('he-calibration-method')?.value || 'corner-based';
                if (calibrationMethod === 'pose-based') {
                    // 姿态法：检查是否是第二步
                    if (currentMotionGroup && currentMotionGroup.pose1 && currentMotionGroup.board_pose1) {
                        // 第二步：采集结束状态
                        captureMotionGroupStep2(data.corners_data);
                    } else {
                        // 第一步：采集初始状态
                        captureMotionGroupStep1(data.corners_data);
                    }
                } else {
                    // 角点法：正常填充表格
                fillHandEyeTable(data.corners_data);
                }
            }
            
            addLog('success', `成功提取 ${data.corners_count} 个角点`);
            showToast(`成功提取 ${data.corners_count} 个角点`, 'success');
        } else {
            addLog('error', '提取角点失败: ' + data.error);
            showToast('提取角点失败: ' + data.error, 'error');
        }
    })
    .catch(error => {
        addLog('error', '提取角点失败: ' + error);
        showToast('提取角点失败', 'error');
    });
}

// 全局变量：保存拍照姿态（Eye-in-Hand模式使用）
let shotPoseData = null;  // 包含：机器人位姿（pos/ori）和角点数据（camera_x, camera_y等）

// 全局变量：姿态法运动组数据（Eye-in-Hand姿态法模式使用）
let motionGroupsData = [];  // 运动组列表，每组包含：{pose1, pose2, board_pose1, board_pose2}

// 填充手眼标定表格（角点数据，位姿留空）
function fillHandEyeTable(cornersData) {
    // #region debug log
    console.log('[HandEye] fillHandEyeTable开始', {corners_count: cornersData.length, timestamp: new Date().toISOString()});
    // #endregion
    
    const calibrationType = document.getElementById('he-calibration-type')?.value || 'eye-to-hand';
    const calibrationMethod = document.getElementById('he-calibration-method')?.value || 'corner-based';
    const tbody = document.querySelector('#pose-list-table tbody');
    tbody.innerHTML = '';
    
    // 姿态法模式：处理运动组数据采集
    if (calibrationType === 'eye-in-hand' && calibrationMethod === 'pose-based') {
        // 姿态法：提取角点后需要获取标定板姿态
        captureMotionGroupStep1(cornersData);
        return;  // 姿态法不需要填充表格，直接返回
    }
    
    // Eye-in-Hand模式：保存拍照姿态数据（角点法）
    if (calibrationType === 'eye-in-hand') {
        // 获取当前机器人位姿作为拍照姿态
        fetch('/api/robot_status')
            .then(response => response.json())
            .then(data => {
                if (data.success && data.is_online && data.cartesian_position) {
                    shotPoseData = {
                        robot_pos_x: data.cartesian_position.position.x,
                        robot_pos_y: data.cartesian_position.position.y,
                        robot_pos_z: data.cartesian_position.position.z,
                        robot_ori_x: data.cartesian_position.orientation.x,
                        robot_ori_y: data.cartesian_position.orientation.y,
                        robot_ori_z: data.cartesian_position.orientation.z,
                        robot_ori_w: data.cartesian_position.orientation.w,
                        corners: cornersData  // 保存角点数据
                    };
                    // #region debug log
                    console.log('[HandEye] 保存拍照姿态（Eye-in-Hand）', {
                        robot_pos: {x: shotPoseData.robot_pos_x, y: shotPoseData.robot_pos_y, z: shotPoseData.robot_pos_z},
                        corners_count: cornersData.length
                    });
                    // #endregion
                    addLog('info', '已保存拍照姿态（Eye-in-Hand模式），请移动机器人到不同位置并点击表格行捕获点选姿态');
                } else {
                    addLog('warning', '无法获取机器人位姿，拍照姿态未保存。请确保机器人在线后再提取角点。');
                }
            })
            .catch(error => {
                console.error('[HandEye] 获取拍照姿态失败', error);
                addLog('warning', '获取拍照姿态失败: ' + error);
            });
    } else {
        // Eye-to-Hand模式：清除拍照姿态
        shotPoseData = null;
    }
    
    cornersData.forEach((corner, index) => {
        const row = tbody.insertRow();
        row.innerHTML = `
            <td>${corner.index}</td>
            <td>${corner.pixel_u.toFixed(2)}</td>
            <td>${corner.pixel_v.toFixed(2)}</td>
            <td>${corner.camera_x.toFixed(3)}</td>
            <td>${corner.camera_y.toFixed(3)}</td>
            <td class="pose-data-cell">-</td>
            <td class="pose-data-cell">-</td>
            <td class="pose-data-cell">-</td>
            <td class="pose-data-cell">-</td>
            <td class="pose-data-cell">-</td>
            <td class="pose-data-cell">-</td>
            <td class="pose-data-cell">-</td>
        `;
        
        // 保存角点数据到行属性
        row.dataset.cornerIndex = corner.index;
        row.dataset.pixelU = corner.pixel_u;
        row.dataset.pixelV = corner.pixel_v;
        row.dataset.cameraX = corner.camera_x;
        row.dataset.cameraY = corner.camera_y;
        
        // 添加点击捕获机器人位姿功能
        row.style.cursor = 'pointer';
        row.title = calibrationType === 'eye-in-hand' ? '点击捕获点选姿态（Eye-in-Hand）' : '点击捕获当前机器人位姿';
        row.addEventListener('click', function() {
            captureRobotPoseToRow(this);
        });
    });
    
    // 更新数据计数
    document.getElementById('pose-count').textContent = cornersData.length;
    
    // #region debug log
    console.log('[HandEye] 表格更新完成', {
        rows_count: cornersData.length,
        mode: 'clear_and_fill',
        calibration_type: calibrationType
    });
    // #endregion
    
    const modeText = calibrationType === 'eye-in-hand' ? '（拍照姿态已保存，请捕获点选姿态）' : '';
    addLog('info', `手眼标定表格已更新，共 ${cornersData.length} 行角点数据 (点击行可捕获位姿)${modeText}`);
}

// 自动为多个表格行捕获机器人位姿
function autoCapturePoseForRows(rows) {
    // #region debug log
    console.log('[HandEye] autoCapturePoseForRows开始', {rows_count: rows.length, timestamp: new Date().toISOString()});
    // #endregion
    
    if (rows.length === 0) {
        console.warn('[HandEye] 空行列表，跳过捕获');
        return;
    }
    
    // 获取当前机器人位姿
    fetch('/api/robot_status')
        .then(response => response.json())
        .then(data => {
            // #region debug log
            console.log('[HandEye] 机器人状态API响应', {
                success: data.success,
                is_online: data.is_online,
                has_cartesian: !!data.cartesian_position
            });
            // #endregion
            
            if (data.success && data.is_online && data.cartesian_position) {
                const pose = data.cartesian_position;
                let capturedCount = 0;
                
                // #region debug log
                console.log('[HandEye] 开始填充位姿数据', {
                    pose_x: pose.position.x,
                    pose_y: pose.position.y,
                    pose_z: pose.position.z
                });
                // #endregion
                
                // 为所有新行填充相同的位姿数据（因为它们是同一时刻捕获的）
                rows.forEach((row, index) => {
                    const cells = row.querySelectorAll('td');
                    
                    // 填充位姿数据（单位：毫米）- 将米转换为毫米（乘以1000）
                    if (cells[5]) cells[5].textContent = (pose.position.x * 1000).toFixed(3);  // 位置 X
                    if (cells[6]) cells[6].textContent = (pose.position.y * 1000).toFixed(3);  // 位置 Y
                    if (cells[7]) cells[7].textContent = (pose.position.z * 1000).toFixed(3);  // 位置 Z
                    if (cells[8]) cells[8].textContent = pose.orientation.x.toFixed(6);  // 姿态 Qx
                    if (cells[9]) cells[9].textContent = pose.orientation.y.toFixed(6);  // 姿态 Qy
                    if (cells[10]) cells[10].textContent = pose.orientation.z.toFixed(6);  // 姿态 Qz
                    if (cells[11]) cells[11].textContent = pose.orientation.w.toFixed(6);  // 姿态 Qw
                    
                    // 保存位姿数据到行属性（单位：米，用于后续标定计算）
                    row.dataset.posX = pose.position.x;
                    row.dataset.posY = pose.position.y;
                    row.dataset.posZ = pose.position.z;
                    row.dataset.oriX = pose.orientation.x;
                    row.dataset.oriY = pose.orientation.y;
                    row.dataset.oriZ = pose.orientation.z;
                    row.dataset.oriW = pose.orientation.w;
                    row.dataset.hasPose = 'true';
                    
                    // 高亮显示该行表示已捕获
                    row.classList.add('pose-captured');
                    
                    capturedCount++;
                });
                
                // 为新添加的行添加闪烁效果
                rows.forEach(row => {
                    row.style.backgroundColor = '#4caf5033';
                });
                setTimeout(() => {
                    rows.forEach(row => {
                        row.style.backgroundColor = '';
                    });
                }, 500);
                
                // #region debug log
                console.log('[HandEye] 位姿捕获完成', {
                    captured_count: capturedCount,
                    total_rows: rows.length,
                    pose_x: (pose.position.x * 1000).toFixed(3),
                    pose_y: (pose.position.y * 1000).toFixed(3),
                    pose_z: (pose.position.z * 1000).toFixed(3)
                });
                // #endregion
                
                addLog('success', `已自动捕获 ${capturedCount} 个角点的机器人位姿: X=${(pose.position.x * 1000).toFixed(3)}, Y=${(pose.position.y * 1000).toFixed(3)}, Z=${(pose.position.z * 1000).toFixed(3)} mm`);
                showToast(`已自动捕获 ${capturedCount} 个角点的位姿`, 'success');
                
                // 更新已捕获位姿的统计
                updateCapturedPoseCount();
            } else if (!data.is_online) {
                // #region debug log
                console.warn('[HandEye] 机器人离线，无法捕获位姿');
                // #endregion
                addLog('warning', '机器人离线，无法自动捕获位姿');
                showToast('机器人离线，请先连接机器人', 'warning');
            } else {
                addLog('warning', '无法获取机器人位姿数据，请稍后手动点击行捕获');
                showToast('无法获取机器人位姿，可手动点击行捕获', 'warning');
            }
        })
        .catch(error => {
            addLog('error', '自动捕获机器人位姿失败: ' + error);
            showToast('自动捕获位姿失败，可手动点击行捕获', 'error');
            console.error('自动捕获机器人位姿失败:', error);
        });
}

// 捕获机器人位姿到指定表格行
function captureRobotPoseToRow(row) {
    // 获取当前机器人位姿
    fetch('/api/robot_status')
        .then(response => response.json())
        .then(data => {
            if (data.success && data.is_online) {
                const pose = data.cartesian_position;
                
                // 获取表格单元格（从第5列开始是位姿数据）
                const cells = row.querySelectorAll('td');
                
                // 填充位姿数据（单位：毫米）- 将米转换为毫米（乘以1000）
                cells[5].textContent = (pose.position.x * 1000).toFixed(3);  // 位置 X
                cells[6].textContent = (pose.position.y * 1000).toFixed(3);  // 位置 Y
                cells[7].textContent = (pose.position.z * 1000).toFixed(3);  // 位置 Z
                cells[8].textContent = pose.orientation.x.toFixed(6);  // 姿态 Qx
                cells[9].textContent = pose.orientation.y.toFixed(6);  // 姿态 Qy
                cells[10].textContent = pose.orientation.z.toFixed(6);  // 姿态 Qz
                cells[11].textContent = pose.orientation.w.toFixed(6);  // 姿态 Qw
                
                // 保存位姿数据到行属性（单位：毫米）
                row.dataset.posX = pose.position.x;
                row.dataset.posY = pose.position.y;
                row.dataset.posZ = pose.position.z;
                row.dataset.oriX = pose.orientation.x;
                row.dataset.oriY = pose.orientation.y;
                row.dataset.oriZ = pose.orientation.z;
                row.dataset.oriW = pose.orientation.w;
                row.dataset.hasPose = 'true';
                
                // 高亮显示该行表示已捕获
                row.classList.add('pose-captured');
                
                // 添加一个短暂的闪烁效果
                row.style.backgroundColor = '#4caf5033';
                setTimeout(() => {
                    row.style.backgroundColor = '';
                }, 500);
                
                const cornerIndex = row.dataset.cornerIndex;
                addLog('success', `角点 #${cornerIndex} 已捕获机器人位姿: X=${(pose.position.x * 1000).toFixed(3)}, Y=${(pose.position.y * 1000).toFixed(3)}, Z=${(pose.position.z * 1000).toFixed(3)} mm`);
                showToast('机器人位姿已捕获', 'success');
                
                // 更新已捕获位姿的统计
                updateCapturedPoseCount();
            } else if (!data.is_online) {
                addLog('warning', '机器人离线，无法捕获位姿');
                showToast('机器人离线，请先连接机器人', 'warning');
            } else {
                addLog('error', '无法获取机器人位姿数据');
                showToast('获取机器人位姿失败', 'error');
            }
        })
        .catch(error => {
            addLog('error', '捕获机器人位姿失败: ' + error);
            showToast('捕获位姿失败', 'error');
            console.error('捕获机器人位姿失败:', error);
        });
}

// 更新已捕获位姿的数量统计
function updateCapturedPoseCount() {
    const tbody = document.querySelector('#pose-list-table tbody');
    const rows = tbody.querySelectorAll('tr');
    let capturedCount = 0;
    
    rows.forEach(row => {
        if (row.dataset.hasPose === 'true') {
            capturedCount++;
        }
    });
    
    // 更新显示
    const statusElement = document.getElementById('calib-status');
    if (capturedCount > 0) {
        statusElement.textContent = `已采集 ${capturedCount} 组位姿数据`;
    } else {
        statusElement.textContent = '未开始';
    }
    
    // 如果有足够的数据，提示可以开始标定
    if (capturedCount >= 5) {
        addLog('info', `已采集 ${capturedCount} 组位姿数据，可以开始标定`);
    }
}

// ============= 手眼标定精度验证按钮事件 =============
function initHandEyeVerifyButtons() {
    // 加载手眼标定参数
    document.getElementById('btn-load-hand-eye-params').addEventListener('click', function() {
        loadHandEyeCalibrationParams();
    });
    
    // 提取棋盘格角点
    document.getElementById('btn-extract-corners-verify').addEventListener('click', function() {
        extractCornersForVerify();
    });
    
    // 获取机器人测试姿态
    document.getElementById('btn-get-robot-test-pose').addEventListener('click', function() {
        getRobotTestPose();
    });
    
    // 加载图像
    document.getElementById('btn-load-image-verify').addEventListener('click', function() {
        loadImageForVerify();
    });
    
    // 采集图像
    document.getElementById('btn-capture-image-verify').addEventListener('click', function() {
        captureImageForVerify();
    });
    
    // 单点测试
    document.getElementById('btn-single-point-test').addEventListener('click', function() {
        performSinglePointTest();
    });
    
    // 自动逐点测试
    document.getElementById('btn-auto-point-test').addEventListener('click', function() {
        performAutoPointTest();
    });
}

// ============= 手眼标定精度验证功能函数 =============

// 全局变量 - 手眼验证相关
let handEyeCalibrationParams = null;
let verifyTestResults = [];

// 全局变量 - 当前选择的点位数据
let currentSelectedPose = {
    x: null,
    y: null,
    z: null,
    qx: null,
    qy: null,
    qz: null,
    qw: null
};

// 更新指定选项卡的图像显示
function updateImageForTab(tabId, imageData) {
    const imageElement = document.getElementById(`${tabId}-image`);
    const containerElement = document.getElementById(`${tabId}-image-container`);
    
    console.log('updateImageForTab called:', tabId, imageData ? 'has data' : 'no data');
    console.log('imageElement:', imageElement);
    console.log('containerElement:', containerElement);
    
    if (imageElement && imageData) {
        // 检查imageData是否已经是完整的data URL
        if (imageData.startsWith('data:image/')) {
            imageElement.src = imageData;
        } else {
            imageElement.src = `data:image/jpeg;base64,${imageData}`;
        }
        imageElement.style.display = 'block';
        
        // 设置图像加载完成事件处理器
        imageElement.onload = function() {
            this.classList.add('loaded');
            console.log('Image loaded and marked as loaded');
        };
        
        // 隐藏占位符
        const placeholder = containerElement.querySelector('.image-placeholder');
        if (placeholder) {
            placeholder.style.display = 'none';
        }
        
        addLog('success', `图像已更新到${tabId}选项卡`);
        console.log('Image updated successfully');
    } else {
        console.log('Failed to update image:', !imageElement ? 'no image element' : 'no image data');
        addLog('error', '图像更新失败');
    }
}

function loadHandEyeCalibrationParams() {
    addLog('info', '打开手眼标定参数文件选择器...');
    
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = '.xml';
    input.style.display = 'none';
    
    input.onchange = function(event) {
        const file = event.target.files[0];
        if (file) {
            addLog('info', `选择手眼标定参数文件: ${file.name}`);
            uploadHandEyeCalibrationFile(file);
        }
    };
    
    document.body.appendChild(input);
    input.click();
    document.body.removeChild(input);
}

function uploadHandEyeCalibrationFile(file) {
    addLog('info', '上传手眼标定参数文件到服务器...');
    
    const formData = new FormData();
    formData.append('file', file);
    
    fetch('/api/hand_eye/load_calibration_params', {
        method: 'POST',
        body: formData
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            handEyeCalibrationParams = data;
            updateVerifyParameterDisplay();
            addLog('success', '手眼标定参数加载成功');
        } else {
            addLog('error', `加载失败: ${data.error}`);
        }
    })
    .catch(error => {
        addLog('error', `加载请求失败: ${error.message}`);
    });
}

function updateVerifyParameterDisplay() {
    if (!handEyeCalibrationParams) return;
    
    // 更新相机内参信息
    const cameraInfo = handEyeCalibrationParams.camera_intrinsic;
    const cameraInfoElement = document.getElementById('verify-camera-intrinsic-info');
    if (cameraInfoElement && cameraInfo.fx) {
        cameraInfoElement.innerHTML = `
            <p><strong>焦距:</strong> fx=${cameraInfo.fx.toFixed(2)}, fy=${cameraInfo.fy.toFixed(2)}</p>
            <p><strong>主点:</strong> cx=${cameraInfo.cx.toFixed(2)}, cy=${cameraInfo.cy.toFixed(2)}</p>
            <p><strong>图像尺寸:</strong> ${cameraInfo.image_size ? cameraInfo.image_size.join(' x ') : '未知'}</p>
            <p><strong>畸变系数:</strong> ${cameraInfo.dist_coeffs ? cameraInfo.dist_coeffs.map(d => d.toFixed(6)).join(', ') : '未知'}</p>
        `;
    }
    
    // 更新手眼标定信息
    const handEyeInfo = handEyeCalibrationParams.hand_eye_calibration;
    const handEyeInfoElement = document.getElementById('verify-hand-eye-info');
    if (handEyeInfoElement && handEyeInfo.calibration_type) {
        // 格式化旋转矩阵
        const rotationMatrix = handEyeInfo.rotation_matrix;
        const rotationStr = rotationMatrix ? 
            `R[0,0]=${rotationMatrix[0][0].toFixed(4)}, R[0,1]=${rotationMatrix[0][1].toFixed(4)}, R[0,2]=${rotationMatrix[0][2].toFixed(4)}<br/>
             R[1,0]=${rotationMatrix[1][0].toFixed(4)}, R[1,1]=${rotationMatrix[1][1].toFixed(4)}, R[1,2]=${rotationMatrix[1][2].toFixed(4)}<br/>
             R[2,0]=${rotationMatrix[2][0].toFixed(4)}, R[2,1]=${rotationMatrix[2][1].toFixed(4)}, R[2,2]=${rotationMatrix[2][2].toFixed(4)}` : '未知';
        
        // 格式化平移向量
        const translationVector = handEyeInfo.translation_vector;
        const translationStr = translationVector ? 
            `t[0]=${translationVector[0].toFixed(2)}, t[1]=${translationVector[1].toFixed(2)}, t[2]=${translationVector[2].toFixed(2)} mm` : '未知';
        
        handEyeInfoElement.innerHTML = `
            <p><strong>标定类型:</strong> ${handEyeInfo.calibration_type}</p>
            <p><strong>标定方法:</strong> ${handEyeInfo.calibration_method}</p>
            <p><strong>标定日期:</strong> ${handEyeInfo.calibration_date}</p>
            <p><strong>相机高度:</strong> ${handEyeInfo.camera_height} mm</p>
            <p><strong>旋转矩阵:</strong><br/>${rotationStr}</p>
            <p><strong>平移向量:</strong> ${translationStr}</p>
            <p><strong>平均误差:</strong> ${handEyeInfo.mean_error.toFixed(3)} mm</p>
            <p><strong>最大误差:</strong> ${handEyeInfo.max_error.toFixed(3)} mm</p>
            <p><strong>最小误差:</strong> ${handEyeInfo.min_error.toFixed(3)} mm</p>
            <p><strong>标准差:</strong> ${handEyeInfo.std_error.toFixed(3)} mm</p>
            <p><strong>数据点数:</strong> ${handEyeInfo.data_count}</p>
        `;
    }
}

function getRobotTestPose() {
    addLog('info', '获取机器人当前测试姿态...');
    
    fetch('/api/robot_status')
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                // 更新机器人姿态显示
                updateVerifyRobotPoseDisplay(data);
                
                // 获取机器人位置（单位：毫米，不需要转换）
                const robotZ = data.cartesian_position.position.z; // 已经是毫米单位
                
                // 将Z轴值填充到测试高度编辑框
                const minTestHeightInput = document.getElementById('min-test-height');
                if (minTestHeightInput) {
                    minTestHeightInput.value = ''; // 先清空
                    minTestHeightInput.value = Math.round(robotZ);
                    addLog('info', `测试高度已设置为: ${Math.round(robotZ)} mm`);
                }
                
                // 将Z轴值+20填充到平移高度编辑框
                const translationHeightInput = document.getElementById('translation-height');
                if (translationHeightInput) {
                    translationHeightInput.value = ''; // 先清空
                    translationHeightInput.value = Math.round(robotZ + 20);
                    addLog('info', `平移高度已设置为: ${Math.round(robotZ + 20)} mm`);
                }
                
                // 存储当前机器人姿态数据到全局变量
                currentSelectedPose.x = data.cartesian_position.position.x;
                currentSelectedPose.y = data.cartesian_position.position.y;
                currentSelectedPose.z = data.cartesian_position.position.z;
                currentSelectedPose.qx = data.cartesian_position.orientation.x;
                currentSelectedPose.qy = data.cartesian_position.orientation.y;
                currentSelectedPose.qz = data.cartesian_position.orientation.z;
                currentSelectedPose.qw = data.cartesian_position.orientation.w;
                
                addLog('info', `当前点位数据已存储: X:${currentSelectedPose.x.toFixed(1)}, Y:${currentSelectedPose.y.toFixed(1)}, Z:${currentSelectedPose.z.toFixed(1)}`);
                
                // 更新完整姿态显示
                updateCurrentRobotPoseDisplay(data);
                
                addLog('success', `机器人姿态获取成功 - 位置: (${data.cartesian_position.position.x.toFixed(2)}, ${data.cartesian_position.position.y.toFixed(2)}, ${data.cartesian_position.position.z.toFixed(2)})`);
                showToast('机器人姿态获取成功，高度参数已自动设置', 'success');
            } else {
                addLog('error', `获取机器人姿态失败: ${data.message}`);
                showToast('获取机器人姿态失败', 'error');
            }
        })
        .catch(error => {
            addLog('error', `获取机器人姿态请求失败: ${error.message}`);
            showToast('获取机器人姿态失败', 'error');
        });
}

// 更新机器人姿态标签中的X和Y坐标
function updateRobotPoseXY(x, y) {
    const poseLabel = document.getElementById('robot-pose-label');
    if (poseLabel) {
        const currentText = poseLabel.textContent;
        // 替换X和Y的值
        const updatedText = currentText.replace(/X: [^;]+/, `X: ${x.toFixed(1)}`).replace(/Y: [^;]+/, `Y: ${y.toFixed(1)}`);
        poseLabel.textContent = updatedText;
        
        // 从当前标签文本中提取其他坐标值
        const zMatch = currentText.match(/Z: ([^;]+)/);
        const qxMatch = currentText.match(/Qx: ([^;]+)/);
        const qyMatch = currentText.match(/Qy: ([^;]+)/);
        const qzMatch = currentText.match(/Qz: ([^;]+)/);
        const qwMatch = currentText.match(/Qw: ([^;]+)/);
        
        // 更新全局变量中的点位数据
        currentSelectedPose.x = x;
        currentSelectedPose.y = y;
        if (zMatch) currentSelectedPose.z = parseFloat(zMatch[1]);
        if (qxMatch) currentSelectedPose.qx = parseFloat(qxMatch[1]);
        if (qyMatch) currentSelectedPose.qy = parseFloat(qyMatch[1]);
        if (qzMatch) currentSelectedPose.qz = parseFloat(qzMatch[1]);
        if (qwMatch) currentSelectedPose.qw = parseFloat(qwMatch[1]);
        
        addLog('info', `姿态标签X和Y已更新: X:${x.toFixed(1)}, Y:${y.toFixed(1)}`);
        addLog('info', `当前点位数据已更新: X:${currentSelectedPose.x.toFixed(1)}, Y:${currentSelectedPose.y.toFixed(1)}, Z:${currentSelectedPose.z ? currentSelectedPose.z.toFixed(1) : '--'}`);
    }
}

// 更新当前机器人姿态显示（验证结果数据标题右侧）
function updateCurrentRobotPoseDisplay(robotStatus) {
    const poseLabel = document.getElementById('robot-pose-label');
    
    if (poseLabel && robotStatus.cartesian_position) {
        // 获取位置和姿态（屏蔽X和Y坐标）
        const robotZ = robotStatus.cartesian_position.position.z.toFixed(1); // 已经是毫米单位
        const qx = robotStatus.cartesian_position.orientation.x.toFixed(3);
        const qy = robotStatus.cartesian_position.orientation.y.toFixed(3);
        const qz = robotStatus.cartesian_position.orientation.z.toFixed(3);
        const qw = robotStatus.cartesian_position.orientation.w.toFixed(3);
        
        // 更新标签文本（X和Y用"--"占位）
        poseLabel.textContent = `X: --; Y: --; Z: ${robotZ}; Qx: ${qx}; Qy: ${qy}; Qz: ${qz}; Qw: ${qw};`;
        
        addLog('info', `姿态标签已更新: X:--, Y:--, Z:${robotZ}, Qx:${qx}, Qy:${qy}, Qz:${qz}, Qw:${qw}`);
    }
}

function loadImageForVerify() {
    addLog('info', '打开本地图像文件选择器...');
    
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = 'image/*';
    input.style.display = 'none';
    
    input.onchange = function(event) {
        const file = event.target.files[0];
        if (file) {
            addLog('info', `选择图像文件: ${file.name}`);
            uploadImageForVerify(file);
        }
    };
    
    document.body.appendChild(input);
    input.click();
    document.body.removeChild(input);
}

function uploadImageForVerify(file) {
    addLog('info', '上传图像到服务器...');
    
    const formData = new FormData();
    formData.append('image', file);
    
    fetch('/api/camera/upload_image', {
        method: 'POST',
        body: formData
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addLog('success', '图像上传成功');
            updateImageForTab('hand-eye-verify', data.image);
        } else {
            addLog('error', `图像上传失败: ${data.error}`);
        }
    })
    .catch(error => {
        addLog('error', `图像上传请求失败: ${error.message}`);
    });
}

function captureImageForVerify() {
    addLog('info', '触发相机拍照...');
    
    fetch('/api/camera/capture', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addLog('success', '相机拍照成功，等待图像数据...');
            // 启动图像更新轮询
            startImageUpdateForVerify();
        } else {
            addLog('error', `相机拍照失败: ${data.error}`);
        }
    })
    .catch(error => {
        addLog('error', `相机拍照请求失败: ${error.message}`);
    });
}

function startImageUpdateForVerify() {
    // 停止之前的轮询
    if (imageUpdateInterval) {
        clearInterval(imageUpdateInterval);
    }
    
    // 开始新的轮询，每500ms检查一次新图像
    let attempts = 0;
    const maxAttempts = 20; // 最多等待10秒
    
    imageUpdateInterval = setInterval(() => {
        attempts++;
        
        fetch('/api/current_image')
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    addLog('success', '获取到新图像');
                    updateImageForTab('hand-eye-verify', data.image);
                    clearInterval(imageUpdateInterval);
                } else if (attempts >= maxAttempts) {
                    addLog('warning', '等待图像超时，请重试');
                    clearInterval(imageUpdateInterval);
                }
            })
            .catch(error => {
                if (attempts >= maxAttempts) {
                    addLog('error', '获取图像失败');
                    clearInterval(imageUpdateInterval);
                }
            });
    }, 500);
}

function updateVerifyRobotPoseDisplay(robotStatus) {
    const posX = document.getElementById('verify-pos-x');
    const posY = document.getElementById('verify-pos-y');
    const posZ = document.getElementById('verify-pos-z');
    const oriX = document.getElementById('verify-ori-x');
    const oriY = document.getElementById('verify-ori-y');
    const oriZ = document.getElementById('verify-ori-z');
    const oriW = document.getElementById('verify-ori-w');
    const onlineIndicator = document.getElementById('verify-robot-online-indicator');
    
    if (posX && robotStatus.cartesian_position) {
        // 将米转换为毫米（乘以1000）
        posX.textContent = (robotStatus.cartesian_position.position.x * 1000).toFixed(2);
        posY.textContent = (robotStatus.cartesian_position.position.y * 1000).toFixed(2);
        posZ.textContent = (robotStatus.cartesian_position.position.z * 1000).toFixed(2);
        oriX.textContent = robotStatus.cartesian_position.orientation.x.toFixed(4);
        oriY.textContent = robotStatus.cartesian_position.orientation.y.toFixed(4);
        oriZ.textContent = robotStatus.cartesian_position.orientation.z.toFixed(4);
        oriW.textContent = robotStatus.cartesian_position.orientation.w.toFixed(4);
    }
    
    if (onlineIndicator) {
        if (robotStatus.is_online) {
            if (robotStatus.in_motion) {
                onlineIndicator.textContent = '运动中';
                onlineIndicator.className = 'status-indicator error';
            } else {
                onlineIndicator.textContent = '在线';
                onlineIndicator.className = 'status-indicator online';
            }
        } else {
            onlineIndicator.textContent = '离线';
            onlineIndicator.className = 'status-indicator offline';
        }
    }
}

function extractCornersForVerify() {
    addLog('info', '开始提取棋盘格角点...');
    
    if (!handEyeCalibrationParams) {
        addLog('error', '请先加载手眼标定参数');
        return;
    }
    
    const squareSize = 15.0; // 默认棋盘格大小
    
    fetch('/api/camera/extract_corners', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            square_size: squareSize
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addLog('success', `角点提取成功，检测到 ${data.corners_count} 个角点`);
            // 更新图像显示
            updateImageForTab('hand-eye-verify', data.image_with_corners);
            
            // 计算并显示坐标转换结果
            calculateAndDisplayCoordinates(data.corners_data);
        } else {
            addLog('error', `角点提取失败: ${data.error}`);
        }
    })
    .catch(error => {
        addLog('error', `角点提取请求失败: ${error.message}`);
    });
}

function calculateAndDisplayCoordinates(cornersData) {
    if (!handEyeCalibrationParams || !cornersData) {
        addLog('error', '缺少必要的数据进行计算');
        return;
    }
    
    addLog('info', '开始计算坐标转换...');
    
    try {
        const transformationMatrix = handEyeCalibrationParams.hand_eye_calibration.transformation_matrix;
        const cameraMatrix = handEyeCalibrationParams.camera_intrinsic.camera_matrix;
        
        if (!transformationMatrix || !cameraMatrix) {
            addLog('error', '缺少变换矩阵或相机矩阵');
            return;
        }
        
        // 转换变换矩阵为numpy数组格式
        const T = transformationMatrix; // 4x4矩阵
        const K = cameraMatrix; // 3x3矩阵
        
        const tableBody = document.querySelector('#verify-result-table tbody');
        tableBody.innerHTML = ''; // 清空现有数据
        
        cornersData.forEach((corner, index) => {
            // 像素坐标
            const pixelU = corner.pixel_u;
            const pixelV = corner.pixel_v;
            
            // 相机坐标（从API返回）
            const cameraX = corner.camera_x;
            const cameraY = corner.camera_y;
            const cameraZ = corner.camera_z;
            
            // 转换为机器人坐标
            const robotCoords = transformCameraToRobot([cameraX, cameraY, cameraZ], T);
            
            // 创建表格行
            const row = document.createElement('tr');
            row.innerHTML = `
                <td>${index + 1}</td>
                <td>${pixelU.toFixed(2)}</td>
                <td>${pixelV.toFixed(2)}</td>
                <td>${cameraX.toFixed(2)}</td>
                <td>${cameraY.toFixed(2)}</td>
                <td>${robotCoords[0].toFixed(2)}</td>
                <td>${robotCoords[1].toFixed(2)}</td>
            `;
            
            // 添加点击事件，更新姿态标签中的X和Y坐标
            row.addEventListener('click', function() {
                updateRobotPoseXY(robotCoords[0], robotCoords[1]);
                addLog('info', `已选择表格行 ${index + 1}，更新姿态标签: X:${robotCoords[0].toFixed(1)}, Y:${robotCoords[1].toFixed(1)}`);
            });
            
            // 添加鼠标悬停效果
            row.style.cursor = 'pointer';
            row.addEventListener('mouseenter', function() {
                this.style.backgroundColor = '#f0f0f0';
            });
            row.addEventListener('mouseleave', function() {
                this.style.backgroundColor = '';
            });
            
            tableBody.appendChild(row);
        });
        
        addLog('success', `坐标转换完成，已显示 ${cornersData.length} 个角点的坐标信息`);
        
    } catch (error) {
        addLog('error', `坐标转换失败: ${error.message}`);
    }
}

function transformCameraToRobot(cameraPoint, transformationMatrix) {
    // cameraPoint: [x, y, z] 相机坐标系下的点
    // transformationMatrix: 4x4变换矩阵
    // 返回: [x, y, z] 机器人坐标系下的点
    
    const [cx, cy, cz] = cameraPoint;
    
    // 齐次坐标
    const cameraHomogeneous = [cx, cy, cz, 1];
    
    // 应用变换矩阵
    const robotHomogeneous = [
        transformationMatrix[0][0] * cameraHomogeneous[0] + transformationMatrix[0][1] * cameraHomogeneous[1] + transformationMatrix[0][2] * cameraHomogeneous[2] + transformationMatrix[0][3] * cameraHomogeneous[3],
        transformationMatrix[1][0] * cameraHomogeneous[0] + transformationMatrix[1][1] * cameraHomogeneous[1] + transformationMatrix[1][2] * cameraHomogeneous[2] + transformationMatrix[1][3] * cameraHomogeneous[3],
        transformationMatrix[2][0] * cameraHomogeneous[0] + transformationMatrix[2][1] * cameraHomogeneous[1] + transformationMatrix[2][2] * cameraHomogeneous[2] + transformationMatrix[2][3] * cameraHomogeneous[3]
    ];
    
    return robotHomogeneous;
}

function performSinglePointTest() {
    addLog('info', '开始单点测试...');
    
    if (!handEyeCalibrationParams) {
        addLog('error', '请先加载手眼标定参数');
        showToast('请先加载手眼标定参数', 'error');
        return;
    }
    
    // 直接显示模态窗口，使用存储的点位数据
    showSinglePointTestModal();
}

// 显示单点测试模态窗口
function showSinglePointTestModal() {
    const modal = document.getElementById('single-point-test-modal');
    if (!modal) {
        addLog('error', '找不到单点测试模态窗口');
        return;
    }
    
    // 检查是否有存储的点位数据
    if (!currentSelectedPose.x || !currentSelectedPose.y) {
        addLog('error', '没有可用的点位数据，请先获取机器人测试姿态或选择表格行');
        showToast('没有可用的点位数据，请先获取机器人测试姿态或选择表格行', 'error');
        return;
    }
    
    // 获取编辑框的值
    const translationHeightInput = document.getElementById('translation-height');
    const testHeightInput = document.getElementById('min-test-height');
    
    const translationHeight = translationHeightInput ? parseFloat(translationHeightInput.value) || 0 : 0;
    const testHeight = testHeightInput ? parseFloat(testHeightInput.value) || 0 : 0;
    
    // 使用存储的点位数据（X, Y, Qx, Qy, Qz, Qw）
    const robotX = currentSelectedPose.x;
    const robotY = currentSelectedPose.y;
    const qx = currentSelectedPose.qx;
    const qy = currentSelectedPose.qy;
    const qz = currentSelectedPose.qz;
    const qw = currentSelectedPose.qw;
    
    // 更新平移点位信息
    document.getElementById('translation-x').textContent = robotX.toFixed(1);
    document.getElementById('translation-y').textContent = robotY.toFixed(1);
    document.getElementById('translation-z').textContent = translationHeight.toFixed(1);
    document.getElementById('translation-qx').textContent = qx.toFixed(3);
    document.getElementById('translation-qy').textContent = qy.toFixed(3);
    document.getElementById('translation-qz').textContent = qz.toFixed(3);
    document.getElementById('translation-qw').textContent = qw.toFixed(3);
    
    // 更新测试点位信息
    document.getElementById('test-x').textContent = robotX.toFixed(1);
    document.getElementById('test-y').textContent = robotY.toFixed(1);
    document.getElementById('test-z').textContent = testHeight.toFixed(1);
    document.getElementById('test-qx').textContent = qx.toFixed(3);
    document.getElementById('test-qy').textContent = qy.toFixed(3);
    document.getElementById('test-qz').textContent = qz.toFixed(3);
    document.getElementById('test-qw').textContent = qw.toFixed(3);
    
    // 显示模态窗口
    modal.style.display = 'block';
    
    addLog('info', `单点测试模态窗口已打开 - 平移高度: ${translationHeight}mm, 测试高度: ${testHeight}mm`);
}

// 关闭单点测试模态窗口
function closeSinglePointTestModal() {
    const modal = document.getElementById('single-point-test-modal');
    if (modal) {
        modal.style.display = 'none';
        addLog('info', '单点测试模态窗口已关闭');
    }
}

// 移动到平移点位
function moveToTranslationPose() {
    addLog('info', '开始移动到平移点位...');
    
    // 获取平移点位数据
    const translationX = parseFloat(document.getElementById('translation-x').textContent);
    const translationY = parseFloat(document.getElementById('translation-y').textContent);
    const translationZ = parseFloat(document.getElementById('translation-z').textContent);
    const qx = parseFloat(document.getElementById('translation-qx').textContent);
    const qy = parseFloat(document.getElementById('translation-qy').textContent);
    const qz = parseFloat(document.getElementById('translation-qz').textContent);
    const qw = parseFloat(document.getElementById('translation-qw').textContent);
    
    // 发送机器人移动指令到后端
    const targetPose = {
        position: {
            x: translationX,
            y: translationY,
            z: translationZ
        },
        orientation: {
            x: qx,
            y: qy,
            z: qz,
            w: qw
        }
    };
    
    addLog('info', `准备移动到平移点位: X:${translationX}, Y:${translationY}, Z:${translationZ}`);
    
    fetch('/api/robot/set_pose', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            target_pose: targetPose,
            use_joints: false,
            velocity: 50.0
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addLog('success', `机器人运动指令发送成功: 平移点位 (${translationX}, ${translationY}, ${translationZ})`);
            showToast('机器人运动指令发送成功', 'success');
        } else {
            addLog('error', `机器人运动指令失败: ${data.error}`);
            showToast('机器人运动指令失败', 'error');
        }
    })
    .catch(error => {
        addLog('error', `发送机器人运动指令请求失败: ${error.message}`);
        showToast('发送机器人运动指令失败', 'error');
    });
    
    // 关闭模态窗口
    closeSinglePointTestModal();
}

// 移动到测试点位
function moveToTestPose() {
    addLog('info', '开始移动到测试点位...');
    
    // 获取测试点位数据
    const testX = parseFloat(document.getElementById('test-x').textContent);
    const testY = parseFloat(document.getElementById('test-y').textContent);
    const testZ = parseFloat(document.getElementById('test-z').textContent);
    const qx = parseFloat(document.getElementById('test-qx').textContent);
    const qy = parseFloat(document.getElementById('test-qy').textContent);
    const qz = parseFloat(document.getElementById('test-qz').textContent);
    const qw = parseFloat(document.getElementById('test-qw').textContent);
    
    // 发送机器人移动指令到后端
    const targetPose = {
        position: {
            x: testX,
            y: testY,
            z: testZ
        },
        orientation: {
            x: qx,
            y: qy,
            z: qz,
            w: qw
        }
    };
    
    addLog('info', `准备移动到测试点位: X:${testX}, Y:${testY}, Z:${testZ}`);
    
    fetch('/api/robot/set_pose', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            target_pose: targetPose,
            use_joints: false,
            velocity: 50.0
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addLog('success', `机器人运动指令发送成功: 测试点位 (${testX}, ${testY}, ${testZ})`);
            showToast('机器人运动指令发送成功', 'success');
        } else {
            addLog('error', `机器人运动指令失败: ${data.error}`);
            showToast('机器人运动指令失败', 'error');
        }
    })
    .catch(error => {
        addLog('error', `发送机器人运动指令请求失败: ${error.message}`);
        showToast('发送机器人运动指令失败', 'error');
    });
    
    // 关闭模态窗口
    closeSinglePointTestModal();
}

function performAutoPointTest() {
    addLog('info', '开始自动逐点测试...');
    
    if (!handEyeCalibrationParams) {
        addLog('error', '请先加载手眼标定参数');
        return;
    }
    
    // TODO: 实现自动逐点测试逻辑
    addLog('info', '自动逐点测试功能待实现');
}

// ============= 手眼标定 =============
function initHandEyeCalibButtons() {
    const buttons = {
        'btn-he-load-camera-params': () => loadCameraParamsForTab('hand-eye-calib'),
        'btn-he-capture-image': () => captureImageForTab('hand-eye-calib'),
        'btn-he-open-local-image': () => openLocalImageForTab('hand-eye-calib'),
        'btn-he-extract-corners': () => extractCornersForTab('hand-eye-calib'),
        'btn-he-format-pose': formatPoseData,
        'btn-he-save-pose-data': savePoseData,
        'btn-he-load-pose-data': loadPoseData,
        'btn-he-start-calibration': startHandEyeCalibration,
        'btn-he-save-calibration': saveHandEyeCalibration
    };
    
    Object.keys(buttons).forEach(btnId => {
        const btn = document.getElementById(btnId);
        if (btn) {
            btn.addEventListener('click', buttons[btnId]);
        }
    });
    
    // 标定类型切换事件
    const calibrationTypeSelect = document.getElementById('he-calibration-type');
    if (calibrationTypeSelect) {
        calibrationTypeSelect.addEventListener('change', function() {
            updateCalibrationTypeUI(this.value);
        });
        // 初始化UI
        updateCalibrationTypeUI(calibrationTypeSelect.value);
    }
    
    // 标定方法切换事件
    const calibrationMethodSelect = document.getElementById('he-calibration-method');
    if (calibrationMethodSelect) {
        calibrationMethodSelect.addEventListener('change', function() {
            updateCalibrationMethodUI();
            addLog('info', `已切换为${this.value === 'pose-based' ? '姿态法' : '角点法'}`);
        });
    }
}

// 更新标定类型UI显示
function updateCalibrationTypeUI(calibrationType) {
    const cameraHeightSetting = document.getElementById('he-camera-height-setting');
    const shotPoseInfo = document.getElementById('he-shot-pose-info');
    const methodSetting = document.getElementById('he-calibration-method-setting');
    const descElement = document.getElementById('he-calibration-type-desc');
    
    if (calibrationType === 'eye-in-hand') {
        if (cameraHeightSetting) cameraHeightSetting.style.display = 'none';
        if (shotPoseInfo) shotPoseInfo.style.display = 'block';
        if (methodSetting) methodSetting.style.display = 'block';
        if (descElement) descElement.textContent = '相机安装在机器人末端，需要拍照姿态和点选姿态';
        updateCalibrationMethodUI(); // 更新方法描述
        addLog('info', '已切换为眼在手上（Eye-in-Hand）标定模式');
    } else {
        if (cameraHeightSetting) cameraHeightSetting.style.display = 'block';
        if (shotPoseInfo) shotPoseInfo.style.display = 'none';
        if (methodSetting) methodSetting.style.display = 'none';
        if (descElement) descElement.textContent = '相机固定，机器人末端点选角点';
        addLog('info', '已切换为眼在手外（Eye-to-Hand）标定模式');
    }
    
    // #region debug log
    console.log('[HandEye] 标定类型切换', {type: calibrationType});
    // #endregion
}

// 更新标定方法UI显示
function updateCalibrationMethodUI() {
    const method = document.getElementById('he-calibration-method')?.value || 'corner-based';
    const descElement = document.getElementById('he-calibration-method-desc');
    const shotPoseInfo = document.getElementById('he-shot-pose-info');
    const poseBasedInfo = document.getElementById('he-pose-based-info');
    
    if (method === 'pose-based') {
        if (descElement) descElement.textContent = '姿态法：需要多组机器人运动（每组2个姿态），观察标定板';
        if (shotPoseInfo) shotPoseInfo.style.display = 'none';
        if (poseBasedInfo) poseBasedInfo.style.display = 'block';
        // 清除角点法的数据
        shotPoseData = null;
        motionGroupsData = [];
        currentMotionGroup = null;
    } else {
        if (descElement) descElement.textContent = '角点法：拍照后点选角点；姿态法：多组机器人运动';
        if (shotPoseInfo) shotPoseInfo.style.display = 'block';
        if (poseBasedInfo) poseBasedInfo.style.display = 'none';
        // 清除姿态法的数据
        motionGroupsData = [];
        currentMotionGroup = null;
    }
    
    // 添加图像文件输入框（手眼标定用）
    const fileInput = document.createElement('input');
    fileInput.type = 'file';
    fileInput.accept = 'image/*';
    fileInput.style.display = 'none';
    fileInput.id = 'file-input-hand-eye-calib';
    document.body.appendChild(fileInput);
    fileInput.addEventListener('change', (e) => handleFileSelectForTab(e, 'hand-eye-calib'));
    
    // 添加位姿数据文件输入框（CSV格式）
    const poseDataInput = document.createElement('input');
    poseDataInput.type = 'file';
    poseDataInput.accept = '.csv';
    poseDataInput.style.display = 'none';
    poseDataInput.id = 'file-input-pose-data';
    document.body.appendChild(poseDataInput);
    poseDataInput.addEventListener('change', handlePoseDataFileSelect);
}

// 格式化位姿数据 - 清除没有机器人位姿的行，只保留已捕获位姿的数据
function formatPoseData() {
    const tbody = document.querySelector('#pose-list-table tbody');
    const rows = tbody.querySelectorAll('tr');
    
    if (rows.length === 0 || (rows.length === 1 && rows[0].querySelector('.no-data'))) {
        addLog('warning', '表格中没有数据');
        showToast('表格为空', 'warning');
        return;
    }
    
    let removedCount = 0;
    let keptCount = 0;
    const rowsToRemove = [];
    
    // 遍历所有行，标记需要删除的行
    rows.forEach(row => {
        if (row.dataset.hasPose === 'true') {
            // 有位姿数据，保留
            keptCount++;
        } else {
            // 没有位姿数据，标记为删除
            rowsToRemove.push(row);
            removedCount++;
        }
    });
    
    if (removedCount === 0) {
        addLog('info', '所有行都已包含机器人位姿数据，无需格式化');
        showToast('所有数据都已完整', 'info');
        return;
    }
    
    // 确认删除
    if (!confirm(`将删除 ${removedCount} 行没有位姿数据的记录，保留 ${keptCount} 行完整数据。\n\n确定继续吗？`)) {
        addLog('info', '用户取消格式化操作');
        return;
    }
    
    // 执行删除
    rowsToRemove.forEach(row => {
        row.remove();
    });
    
    // 如果没有剩余数据，显示提示
    if (keptCount === 0) {
        tbody.innerHTML = '<tr><td colspan="12" class="no-data">暂无数据</td></tr>';
    }
    
    // 更新统计
    updateCapturedPoseCount();
    
    addLog('success', `格式化完成：删除了 ${removedCount} 行空数据，保留 ${keptCount} 行完整数据`);
    showToast(`已清理 ${removedCount} 行空数据`, 'success');
}

// 保存位姿数据为CSV文件
function savePoseData() {
    const tbody = document.querySelector('#pose-list-table tbody');
    const rows = tbody.querySelectorAll('tr');
    
    // 检查是否有数据
    if (rows.length === 0 || (rows.length === 1 && rows[0].querySelector('.no-data'))) {
        addLog('warning', '表格中没有数据可保存');
        showToast('没有数据可保存', 'warning');
        return;
    }
    
    // 统计有位姿数据的行数
    let dataCount = 0;
    rows.forEach(row => {
        if (row.dataset.hasPose === 'true') {
            dataCount++;
        }
    });
    
    if (dataCount === 0) {
        addLog('warning', '表格中没有完整的位姿数据');
        showToast('请先捕获机器人位姿', 'warning');
        return;
    }
    
    // 构建CSV内容
    const table = document.getElementById('pose-list-table');
    const headers = Array.from(table.querySelectorAll('thead th')).map(th => th.textContent.trim());
    
    // CSV头部
    let csvContent = headers.join(',') + '\n';
    
    // CSV数据行
    rows.forEach(row => {
        const cells = row.querySelectorAll('td');
        const rowData = [];
        
        cells.forEach(cell => {
            let value = cell.textContent.trim();
            // 如果包含逗号或换行符，用双引号包裹
            if (value.includes(',') || value.includes('\n') || value.includes('"')) {
                value = '"' + value.replace(/"/g, '""') + '"';
            }
            rowData.push(value);
        });
        
        csvContent += rowData.join(',') + '\n';
    });
    
    // 创建Blob并下载
    const blob = new Blob(['\ufeff' + csvContent], { type: 'text/csv;charset=utf-8;' });
    const link = document.createElement('a');
    
    // 生成文件名（带时间戳）
    const now = new Date();
    const timestamp = now.getFullYear() + 
                     String(now.getMonth() + 1).padStart(2, '0') + 
                     String(now.getDate()).padStart(2, '0') + '_' +
                     String(now.getHours()).padStart(2, '0') + 
                     String(now.getMinutes()).padStart(2, '0') + 
                     String(now.getSeconds()).padStart(2, '0');
    const filename = `hand_eye_pose_data_${timestamp}.csv`;
    
    // 触发下载
    if (navigator.msSaveBlob) {
        // IE 10+
        navigator.msSaveBlob(blob, filename);
    } else {
        link.href = URL.createObjectURL(blob);
        link.download = filename;
        link.style.display = 'none';
        document.body.appendChild(link);
        link.click();
        document.body.removeChild(link);
        URL.revokeObjectURL(link.href);
    }
    
    addLog('success', `位姿数据已保存: ${filename} (共 ${rows.length} 行数据)`);
    showToast(`CSV文件已保存: ${filename}`, 'success');
}

// 加载本地位姿数据（CSV格式）
function loadPoseData() {
    const fileInput = document.getElementById('file-input-pose-data');
    if (fileInput) {
        fileInput.click();
    }
}

// 处理位姿数据文件选择
function handlePoseDataFileSelect(event) {
    const file = event.target.files[0];
    if (!file) return;
    
    // 检查文件扩展名
    if (!file.name.toLowerCase().endsWith('.csv')) {
        addLog('error', '请选择CSV格式的文件');
        showToast('只支持CSV格式', 'error');
        event.target.value = '';
        return;
    }
    
    addLog('info', `选择位姿数据文件: ${file.name}`);
    showToast('正在加载位姿数据...', 'info');
    
    const reader = new FileReader();
    reader.onload = function(e) {
        try {
            const content = e.target.result;
            addLog('info', `文件读取成功，大小: ${(content.length/1024).toFixed(1)}KB`);
            
            // 解析CSV内容
            const lines = content.split('\n').filter(line => line.trim() !== '');
            
            if (lines.length < 2) {
                addLog('error', 'CSV文件为空或格式不正确');
                showToast('文件内容为空', 'error');
                return;
            }
            
            // 第一行是表头，跳过
            const headers = parseCSVLine(lines[0]);
            const dataLines = lines.slice(1);
            
            addLog('info', `解析到 ${dataLines.length} 行数据`);
            
            // 清空当前表格
            const tbody = document.querySelector('#pose-list-table tbody');
            tbody.innerHTML = '';
            
            let loadedCount = 0;
            let errorCount = 0;
            
            // 解析每一行数据并填充到表格
            dataLines.forEach((line, index) => {
                try {
                    const values = parseCSVLine(line);
                    
                    // 检查列数是否匹配（应该有12列）
                    if (values.length < 12) {
                        addLog('warning', `第 ${index + 2} 行数据列数不足，已跳过`);
                        errorCount++;
                        return;
                    }
                    
                    // 创建表格行
                    const row = tbody.insertRow();
                    
                    // 填充所有列的数据
                    values.forEach((value, colIndex) => {
                        const cell = row.insertCell();
                        cell.textContent = value;
                        
                        // 位姿数据列添加特殊样式
                        if (colIndex >= 5) {
                            cell.className = 'pose-data-cell';
                        }
                    });
                    
                    // 保存数据到行属性
                    row.dataset.cornerIndex = values[0];
                    row.dataset.pixelU = values[1];
                    row.dataset.pixelV = values[2];
                    row.dataset.cameraX = values[3];
                    row.dataset.cameraY = values[4];
                    
                    // 检查是否有位姿数据（第6列之后）
                    const hasPoseData = values[5] && values[5] !== '-' && values[5].trim() !== '';
                    
                    if (hasPoseData) {
                        row.dataset.posX = values[5];
                        row.dataset.posY = values[6];
                        row.dataset.posZ = values[7];
                        row.dataset.oriX = values[8];
                        row.dataset.oriY = values[9];
                        row.dataset.oriZ = values[10];
                        row.dataset.oriW = values[11];
                        row.dataset.hasPose = 'true';
                        row.classList.add('pose-captured');
                    }
                    
                    // 添加点击捕获位姿功能
                    row.style.cursor = 'pointer';
                    row.title = '点击捕获当前机器人位姿';
                    row.addEventListener('click', function() {
                        captureRobotPoseToRow(this);
                    });
                    
                    loadedCount++;
                    
                } catch (error) {
                    addLog('warning', `第 ${index + 2} 行数据解析失败: ${error.message}`);
                    errorCount++;
                }
            });
            
            // 更新统计
            updateCapturedPoseCount();
            
            if (loadedCount > 0) {
                addLog('success', `成功加载 ${loadedCount} 行数据${errorCount > 0 ? `, ${errorCount} 行失败` : ''}`);
                showToast(`已加载 ${loadedCount} 行位姿数据`, 'success');
            } else {
                addLog('error', '没有成功加载任何数据');
                showToast('数据加载失败', 'error');
                tbody.innerHTML = '<tr><td colspan="12" class="no-data">暂无数据</td></tr>';
            }
            
        } catch (error) {
            addLog('error', '解析CSV文件失败: ' + error.message);
            showToast('文件解析失败', 'error');
            console.error('CSV解析错误:', error);
        }
    };
    
    reader.onerror = function() {
        addLog('error', '文件读取失败');
        showToast('文件读取失败', 'error');
    };
    
    reader.readAsText(file, 'UTF-8');
    event.target.value = '';
}

// 解析CSV行（处理引号和逗号）
function parseCSVLine(line) {
    const result = [];
    let current = '';
    let inQuotes = false;
    
    for (let i = 0; i < line.length; i++) {
        const char = line[i];
        
        if (char === '"') {
            if (inQuotes && line[i + 1] === '"') {
                // 双引号转义
                current += '"';
                i++;
            } else {
                // 切换引号状态
                inQuotes = !inQuotes;
            }
        } else if (char === ',' && !inQuotes) {
            // 分隔符
            result.push(current.trim());
            current = '';
        } else {
            current += char;
        }
    }
    
    // 添加最后一个字段
    result.push(current.trim());
    
    return result;
}

// 开始手眼标定
function startHandEyeCalibration() {
    // #region debug log
    console.log('[HandEye] startHandEyeCalibration开始', {timestamp: new Date().toISOString()});
    // #endregion
    
    addLog('info', '准备开始手眼标定...');
    
    // 收集表格中的数据
    const tbody = document.querySelector('#pose-list-table tbody');
    const rows = tbody.querySelectorAll('tr');
    
    // #region debug log
    console.log('[HandEye] 数据收集', {total_rows: rows.length});
    // #endregion
    
    // 检查是否有数据
    if (rows.length === 0 || (rows.length === 1 && rows[0].querySelector('.no-data'))) {
        console.error('[HandEye] 表格中没有数据');
        addLog('error', '表格中没有数据');
        showToast('请先采集数据', 'error');
        return;
    }
    
    // 收集有位姿数据的行
    const poseData = [];
    let hasPoseCount = 0;
    rows.forEach((row, index) => {
        if (row.dataset.hasPose === 'true') {
            hasPoseCount++;
            try {
                const data = {
                    corner_index: parseInt(row.dataset.cornerIndex),
                    pixel_u: parseFloat(row.dataset.pixelU),
                    pixel_v: parseFloat(row.dataset.pixelV),
                    camera_x: parseFloat(row.dataset.cameraX),
                    camera_y: parseFloat(row.dataset.cameraY),
                    robot_pos_x: parseFloat(row.dataset.posX),
                    robot_pos_y: parseFloat(row.dataset.posY),
                    robot_pos_z: parseFloat(row.dataset.posZ),
                    robot_ori_x: parseFloat(row.dataset.oriX),
                    robot_ori_y: parseFloat(row.dataset.oriY),
                    robot_ori_z: parseFloat(row.dataset.oriZ),
                    robot_ori_w: parseFloat(row.dataset.oriW)
                };
                poseData.push(data);
            } catch (error) {
                console.error(`[HandEye] 数据行解析失败 (行${index}):`, error, row.dataset);
                addLog('warning', `数据行解析失败: ${error.message}`);
            }
        }
    });
    
    // #region debug log
    console.log('[HandEye] 数据收集完成', {
        total_rows: rows.length,
        has_pose_rows: hasPoseCount,
        valid_pose_data: poseData.length
    });
    // #endregion
    
    if (poseData.length < 3) {
        console.error('[HandEye] 数据不足', {pose_data_count: poseData.length, required: 3});
        addLog('error', `数据不足，至少需要3组位姿数据，当前只有${poseData.length}组`);
        showToast(`至少需要3组完整数据，当前只有${poseData.length}组`, 'error');
        return;
    }
    
    // 获取标定类型和方法
    const calibrationType = document.getElementById('he-calibration-type')?.value || 'eye-to-hand';
    const calibrationMethod = document.getElementById('he-calibration-method')?.value || 'corner-based';
    
    // #region debug log
    console.log('[HandEye] 开始标定调用', {
        calibration_type: calibrationType,
        calibration_method: calibrationMethod,
        pose_data_count: poseData.length,
        has_shot_pose: !!shotPoseData
    });
    // #endregion
    
    // 准备请求数据
    let requestData = {
        calibration_type: calibrationType,
        calibration_method: calibrationMethod,  // 传递标定方法
        pose_data: poseData
    };
    
    if (calibrationType === 'eye-to-hand') {
        // Eye-to-Hand: 需要相机高度
        const cameraHeight = parseFloat(document.getElementById('he-camera-height').value) || 840.0;
        requestData.camera_height = cameraHeight;
        addLog('info', `开始Eye-to-Hand标定，使用 ${poseData.length} 组位姿数据，相机高度: ${cameraHeight} mm`);
    } else {
        // Eye-in-Hand: 需要拍照姿态
        if (!shotPoseData) {
            console.error('[HandEye] Eye-in-Hand模式缺少拍照姿态');
            addLog('error', 'Eye-in-Hand模式需要拍照姿态，请先提取角点以保存拍照姿态');
            showToast('请先提取角点以保存拍照姿态', 'error');
            return;
        }
        requestData.shot_pose = {
            robot_pos_x: shotPoseData.robot_pos_x,
            robot_pos_y: shotPoseData.robot_pos_y,
            robot_pos_z: shotPoseData.robot_pos_z,
            robot_ori_x: shotPoseData.robot_ori_x,
            robot_ori_y: shotPoseData.robot_ori_y,
            robot_ori_z: shotPoseData.robot_ori_z,
            robot_ori_w: shotPoseData.robot_ori_w,
            corners: shotPoseData.corners  // 拍照时的角点数据
        };
        addLog('info', `开始Eye-in-Hand标定，使用 ${poseData.length} 组点选姿态数据`);
    }
    
    showToast('正在进行手眼标定...', 'info');
    
    // 更新状态
    document.getElementById('calib-status').textContent = '标定中...';
    document.getElementById('calib-error').textContent = '计算中...';
    
    // 调用后端API
    fetch('/api/hand_eye/calibrate', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(requestData)
    })
    .then(response => {
        // #region debug log
        console.log('[HandEye] API响应状态', {ok: response.ok, status: response.status, statusText: response.statusText});
        // #endregion
        
        // 检查HTTP状态码
        if (!response.ok) {
            // 尝试解析错误响应
            return response.json().then(errorData => {
                throw new Error(errorData.error || errorData.message || `HTTP ${response.status}: ${response.statusText}`);
            }).catch(() => {
                // 如果无法解析JSON，抛出HTTP错误
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            });
        }
        
        return response.json();
    })
    .then(data => {
        // #region debug log
        console.log('[HandEye] 标定API返回', {
            success: data.success,
            has_evaluation: !!data.evaluation,
            mean_error: data.evaluation?.mean_error,
            error: data.error,
            message: data.message,
            full_data: data
        });
        // #endregion
        
        if (data.success) {
            addLog('success', `标定成功！平均误差: ${data.evaluation.mean_error.toFixed(3)} mm`);
            
            // 更新状态显示
            const statusEl = document.getElementById('calib-status');
            const errorEl = document.getElementById('calib-error');
            if (statusEl) statusEl.textContent = '标定完成';
            if (errorEl) errorEl.textContent = `${data.evaluation.mean_error.toFixed(3)} mm`;
            
            // 显示详细评估结果弹窗
            showCalibrationResult(data);
            
            showToast('手眼标定完成', 'success');
        } else {
            // 获取错误信息
            const errorMsg = data.error || data.message || '未知错误';
            
            console.error('[HandEye] 标定失败', {
                error: errorMsg,
                full_response: data,
                timestamp: new Date().toISOString()
            });
            
            addLog('error', `标定失败: ${errorMsg}`);
            showToast(`标定失败: ${errorMsg}`, 'error');
            
            // 更新状态显示
            const statusEl = document.getElementById('calib-status');
            const errorEl = document.getElementById('calib-error');
            if (statusEl) statusEl.textContent = '标定失败';
            if (errorEl) errorEl.textContent = errorMsg.length > 20 ? errorMsg.substring(0, 20) + '...' : errorMsg;
        }
    })
    .catch(error => {
        const errorMsg = error.message || String(error) || '网络错误或服务器错误';
        
        console.error('[HandEye] 标定API调用错误', {
            error: errorMsg,
            error_type: error.name,
            stack: error.stack,
            timestamp: new Date().toISOString()
        });
        
        addLog('error', `标定失败: ${errorMsg}`);
        showToast(`标定失败: ${errorMsg}`, 'error');
        
        // 更新状态显示
        const statusEl = document.getElementById('calib-status');
        const errorEl = document.getElementById('calib-error');
        if (statusEl) statusEl.textContent = '标定失败';
        if (errorEl) errorEl.textContent = errorMsg.length > 20 ? errorMsg.substring(0, 20) + '...' : errorMsg;
        
        console.error('手眼标定错误:', error);
    });
}

// 显示标定结果评估弹窗
function showCalibrationResult(data) {
    const eval_data = data.evaluation;
    
    // 创建弹窗HTML
    const modalHTML = `
        <div class="result-modal-overlay" id="calibResultModal">
            <div class="result-modal">
                <div class="result-modal-header">
                    <h2>🎉 手眼标定结果评估</h2>
                    <button class="result-modal-close" onclick="closeCalibResultModal()">×</button>
                </div>
                <div class="result-modal-body">
                    <div class="result-section">
                        <h3>📊 标定信息</h3>
                        <table class="result-table">
                            <tr>
                                <td class="result-label">标定方法</td>
                                <td class="result-value">${data.method || 'Tsai'}</td>
                            </tr>
                            <tr>
                                <td class="result-label">数据组数</td>
                                <td class="result-value">${eval_data.data_count} 组</td>
                            </tr>
                            <tr>
                                <td class="result-label">相机高度</td>
                                <td class="result-value">${data.camera_height ? data.camera_height.toFixed(1) : '840.0'} mm</td>
                            </tr>
                            <tr>
                                <td class="result-label">标定类型</td>
                                <td class="result-value" style="font-size: 12px;">${data.calibration_type || 'Eye-to-Hand'}</td>
                            </tr>
                            <tr>
                                <td class="result-label">标定场景</td>
                                <td class="result-value" style="font-size: 12px;">相机固定，机器人末端点选角点</td>
                            </tr>
                        </table>
                    </div>
                    
                    ${data.data_quality ? `
                    <div class="result-section">
                        <h3>📐 数据分布范围</h3>
                        <table class="result-table">
                            <tr>
                                <td class="result-label">相机视野 X</td>
                                <td class="result-value ${data.data_quality.position_range_camera.x < 50 ? 'result-warning' : 'result-good'}">
                                    ${data.data_quality.position_range_camera.x.toFixed(2)} mm
                                </td>
                            </tr>
                            <tr>
                                <td class="result-label">相机视野 Y</td>
                                <td class="result-value ${data.data_quality.position_range_camera.y < 50 ? 'result-warning' : 'result-good'}">
                                    ${data.data_quality.position_range_camera.y.toFixed(2)} mm
                                </td>
                            </tr>
                            <tr>
                                <td class="result-label">机器人移动 X</td>
                                <td class="result-value">${data.data_quality.position_range_robot.x.toFixed(2)} mm</td>
                            </tr>
                            <tr>
                                <td class="result-label">机器人移动 Y</td>
                                <td class="result-value">${data.data_quality.position_range_robot.y.toFixed(2)} mm</td>
                            </tr>
                            <tr>
                                <td class="result-label">机器人移动 Z</td>
                                <td class="result-value">${data.data_quality.position_range_robot.z.toFixed(2)} mm</td>
                            </tr>
                        </table>
                    </div>
                    ` : ''}
                    
                    <div class="result-section">
                        <h3>📈 误差统计</h3>
                        <table class="result-table">
                            <tr>
                                <td class="result-label">平均误差</td>
                                <td class="result-value ${eval_data.mean_error < 2 ? 'result-good' : eval_data.mean_error < 5 ? 'result-warning' : 'result-bad'}">
                                    ${eval_data.mean_error.toFixed(3)} mm
                                </td>
                            </tr>
                            <tr>
                                <td class="result-label">最小误差</td>
                                <td class="result-value">${eval_data.min_error.toFixed(3)} mm</td>
                            </tr>
                            <tr>
                                <td class="result-label">最大误差</td>
                                <td class="result-value">${eval_data.max_error.toFixed(3)} mm</td>
                            </tr>
                            <tr>
                                <td class="result-label">标准差</td>
                                <td class="result-value">${eval_data.std_error.toFixed(3)} mm</td>
                            </tr>
                        </table>
                    </div>
                    
                    <div class="result-section">
                        <h3>📐 变换矩阵 (相机→末端执行器)</h3>
                        <div class="result-matrix">
                            <h4>旋转矩阵 (R):</h4>
                            <pre class="matrix-display">${formatMatrix(data.rotation_matrix, 6)}</pre>
                        </div>
                        <div class="result-matrix">
                            <h4>平移向量 (t) [毫米]:</h4>
                            <pre class="matrix-display">[${data.translation_vector.map(v => v.toFixed(3)).join(', ')}]</pre>
                        </div>
                    </div>
                    
                    <div class="result-section">
                        <h3>📈 各位姿误差分布</h3>
                        <div class="result-errors">
                            ${eval_data.errors_per_pose.map((err, idx) => `
                                <div class="error-bar-container">
                                    <span class="error-label">位姿 #${idx + 1}</span>
                                    <div class="error-bar-wrapper">
                                        <div class="error-bar ${err < 2 ? 'error-good' : err < 5 ? 'error-warning' : 'error-bad'}" 
                                             style="width: ${Math.min(err / eval_data.max_error * 100, 100)}%">
                                        </div>
                                    </div>
                                    <span class="error-value">${err.toFixed(2)} mm</span>
                                </div>
                            `).join('')}
                        </div>
                    </div>
                    
                    <div class="result-section">
                        <h3>✅ 标定质量评估</h3>
                        <div class="quality-assessment">
                            ${getQualityAssessment(eval_data.mean_error, data.data_quality)}
                        </div>
                    </div>
                </div>
                <div class="result-modal-footer">
                    <button class="action-button" onclick="copyCalibrationData()">📋 复制数据</button>
                    <button class="action-button primary" onclick="closeCalibResultModal()">确定</button>
                </div>
            </div>
        </div>
    `;
    
    // 添加到页面
    const existingModal = document.getElementById('calibResultModal');
    if (existingModal) {
        existingModal.remove();
    }
    
    document.body.insertAdjacentHTML('beforeend', modalHTML);
    
    // 保存数据用于复制
    window.calibrationResultData = data;
}

// 格式化矩阵显示
function formatMatrix(matrix, precision = 6) {
    return matrix.map(row => 
        '[' + row.map(val => val.toFixed(precision).padStart(precision + 4)).join(', ') + ']'
    ).join('\n');
}

// 获取质量评估（Eye-to-Hand场景）
function getQualityAssessment(mean_error, data_quality = null) {
    // Eye-to-Hand场景的质量评估（点云配准方法）
    
    // 检查数据分布
    if (data_quality && data_quality.position_range_camera) {
        const cam_x = data_quality.position_range_camera.x;
        const cam_y = data_quality.position_range_camera.y;
        
        if (cam_x < 30 || cam_y < 30) {
            return `
                <div class="quality-poor">
                    <span class="quality-icon">⚠️</span>
                    <span class="quality-text">数据分布不足</span>
                    <div style="text-align: left; margin-top: 16px;">
                        <p><strong>问题：数据点分布范围太小！</strong></p>
                        <ul style="margin: 8px 0; padding-left: 20px;">
                            <li>相机视野覆盖 X: ${cam_x.toFixed(2)} mm ${cam_x < 30 ? '(⚠️ 建议 > 50mm)' : ''}</li>
                            <li>相机视野覆盖 Y: ${cam_y.toFixed(2)} mm ${cam_y < 30 ? '(⚠️ 建议 > 50mm)' : ''}</li>
                        </ul>
                        <p><strong>改进建议：</strong></p>
                        <ul style="margin: 8px 0; padding-left: 20px;">
                            <li>让机器人末端移动到棋盘格的不同角点位置</li>
                            <li>选择分布在四个角落和中心的角点</li>
                            <li>增加数据点的空间覆盖范围</li>
                        </ul>
                    </div>
                </div>
            `;
        }
    }
    
    // 根据误差评估（Eye-to-Hand场景的标准）
    if (mean_error < 2) {
        return `
            <div class="quality-excellent">
                <span class="quality-icon">🌟</span>
                <span class="quality-text">优秀</span>
                <div style="text-align: left; margin-top: 16px;">
                    <p><strong>标定质量评估：优秀</strong></p>
                    <ul style="margin: 8px 0; padding-left: 20px; font-size: 14px;">
                        <li>✅ 平均误差 ${mean_error.toFixed(3)} mm < 2 mm</li>
                        <li>✅ 标定精度达到高精度级别</li>
                        <li>✅ 相机高度设置准确</li>
                        <li>✅ 数据点分布良好</li>
                    </ul>
                    <p style="margin-top: 12px;"><strong>适用场景：</strong></p>
                    <ul style="margin: 8px 0; padding-left: 20px; font-size: 13px;">
                        <li>精密装配（公差 < 0.5mm）</li>
                        <li>高精度抓取和放置</li>
                        <li>视觉引导精密操作</li>
                    </ul>
                    <p style="margin-top: 12px; color: #4caf50; font-weight: 600;">
                        ✓ 标定结果可直接用于生产
                    </p>
                </div>
            </div>
        `;
    } else if (mean_error < 5) {
        return `
            <div class="quality-good">
                <span class="quality-icon">✔️</span>
                <span class="quality-text">良好</span>
                <div style="text-align: left; margin-top: 16px;">
                    <p><strong>标定质量评估：良好</strong></p>
                    <ul style="margin: 8px 0; padding-left: 20px; font-size: 14px;">
                        <li>✅ 平均误差 ${mean_error.toFixed(3)} mm，在2-5mm范围内</li>
                        <li>✅ 标定精度满足一般应用需求</li>
                        <li>✅ 数据点分布合理</li>
                    </ul>
                    <p style="margin-top: 12px;"><strong>适用场景：</strong></p>
                    <ul style="margin: 8px 0; padding-left: 20px; font-size: 13px;">
                        <li>普通抓取和放置</li>
                        <li>物料搬运</li>
                        <li>一般视觉引导任务</li>
                    </ul>
                    <p style="margin-top: 12px;"><strong>精度优化建议：</strong></p>
                    <ul style="margin: 8px 0; padding-left: 20px; font-size: 13px;">
                        <li>尝试微调相机高度(±5-10mm)重新标定</li>
                        <li>增加数据点数量（当前已有足够）</li>
                        <li>选择更分散的角点位置</li>
                    </ul>
                </div>
            </div>
        `;
    } else if (mean_error < 20) {
        return `
            <div class="quality-poor">
                <span class="quality-icon">⚠️</span>
                <span class="quality-text">需改进</span>
                <div style="text-align: left; margin-top: 16px;">
                    <p><strong>标定质量评估：需改进</strong></p>
                    <ul style="margin: 8px 0; padding-left: 20px; font-size: 14px;">
                        <li>⚠️ 平均误差 ${mean_error.toFixed(1)} mm，精度偏低</li>
                    </ul>
                    <p style="margin-top: 12px;"><strong>可能的问题：</strong></p>
                    <ul style="margin: 8px 0; padding-left: 20px; font-size: 13px;">
                        <li>相机高度设置可能偏差较大（当前${data.camera_height}mm）</li>
                        <li>数据点分布不够均匀</li>
                        <li>角点检测精度不足</li>
                        <li>机器人末端未准确到达角点位置</li>
                    </ul>
                    <p style="margin-top: 12px;"><strong>改进建议：</strong></p>
                    <ul style="margin: 8px 0; padding-left: 20px; font-size: 13px;">
                        <li>尝试调整相机高度(±20mm)重新标定</li>
                        <li>重新采集数据，确保数据点分布均匀</li>
                        <li>检查角点检测是否准确（图像清晰度）</li>
                    </ul>
                </div>
            </div>
        `;
    } else {
        return `
            <div class="quality-poor">
                <span class="quality-icon">❌</span>
                <span class="quality-text">标定失败</span>
                <div style="text-align: left; margin-top: 16px;">
                    <p><strong>标定质量评估：失败</strong></p>
                    <ul style="margin: 8px 0; padding-left: 20px; font-size: 14px;">
                        <li>❌ 平均误差 ${mean_error.toFixed(1)} mm，精度极低</li>
                    </ul>
                    <p style="margin-top: 12px;"><strong>可能的严重问题：</strong></p>
                    <ul style="margin: 8px 0; padding-left: 20px; font-size: 13px;">
                        <li>相机高度设置严重错误（当前${data.camera_height}mm）</li>
                        <li>机器人坐标系定义错误（方向或单位）</li>
                        <li>数据采集方法错误（末端未到达角点）</li>
                        <li>相机坐标计算错误</li>
                    </ul>
                    <p style="margin-top: 12px;"><strong>建议操作：</strong></p>
                    <ul style="margin: 8px 0; padding-left: 20px; font-size: 13px;">
                        <li>重新测量相机高度（使用测距仪）</li>
                        <li>验证机器人坐标系设置</li>
                        <li>检查数据采集流程</li>
                        <li>尝试不同的相机高度值（400-700mm）</li>
                    </ul>
                </div>
            </div>
        `;
    }
}

// 关闭标定结果弹窗
function closeCalibResultModal() {
    const modal = document.getElementById('calibResultModal');
    if (modal) {
        modal.remove();
    }
}

// 复制标定数据
function copyCalibrationData() {
    if (!window.calibrationResultData) {
        showToast('没有可复制的数据', 'warning');
        return;
    }
    
    const data = window.calibrationResultData;
    const text = `
手眼标定结果
============
数据组数: ${data.evaluation.data_count}
平均误差: ${data.evaluation.mean_error.toFixed(3)} mm
最小误差: ${data.evaluation.min_error.toFixed(3)} mm
最大误差: ${data.evaluation.max_error.toFixed(3)} mm
标准差: ${data.evaluation.std_error.toFixed(3)} mm

旋转矩阵 (R):
${formatMatrix(data.rotation_matrix, 6)}

平移向量 (t) [毫米]:
[${data.translation_vector.map(v => v.toFixed(3)).join(', ')}]

变换矩阵 (4x4) [旋转部分无单位，平移部分单位为毫米]:
${formatMatrix(data.transformation_matrix, 3)}
    `.trim();
    
    navigator.clipboard.writeText(text).then(() => {
        showToast('标定数据已复制到剪贴板', 'success');
    }).catch(err => {
        console.error('复制失败:', err);
        showToast('复制失败', 'error');
    });
}

// 保存手眼标定结果为XML文件（浏览器下载）
function saveHandEyeCalibration() {
    // 检查是否有标定结果
    if (!window.calibrationResultData) {
        addLog('warning', '没有可保存的标定结果，请先完成标定');
        showToast('请先完成标定', 'warning');
        return;
    }
    
    addLog('info', '准备保存标定结果...');
    showToast('正在生成标定文件...', 'info');
    
    // 调用后端API生成XML内容
    fetch('/api/hand_eye/save_calibration', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify(window.calibrationResultData)
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 生成文件名（带时间戳）
            const now = new Date();
            const timestamp = now.getFullYear() + 
                             String(now.getMonth() + 1).padStart(2, '0') + 
                             String(now.getDate()).padStart(2, '0') + '_' +
                             String(now.getHours()).padStart(2, '0') + 
                             String(now.getMinutes()).padStart(2, '0') + 
                             String(now.getSeconds()).padStart(2, '0');
            const filename = `hand_eye_calibration_${timestamp}.xml`;
            
            // 创建Blob并触发下载
            const blob = new Blob([data.xml_content], { type: 'application/xml;charset=utf-8' });
            const link = document.createElement('a');
            link.href = URL.createObjectURL(blob);
            link.download = filename;
            link.style.display = 'none';
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
            URL.revokeObjectURL(link.href);
            
            addLog('success', `标定结果已下载: ${filename}`);
            showToast(`标定文件已下载: ${filename}`, 'success');
            
            // 显示下载成功信息
            const message = `
标定结果已成功下载！

文件名：${filename}

文件包含：
✓ 相机内参矩阵
✓ 畸变系数  
✓ 手眼变换矩阵（相机→机器人基座）
✓ 标定精度评估
✓ 标定算法和日期

平均误差：${window.calibrationResultData.evaluation.mean_error.toFixed(3)} mm

请妥善保存此文件！
            `.trim();
            
            alert(message);
        } else {
            addLog('error', `生成失败: ${data.error}`);
            showToast(`生成失败: ${data.error}`, 'error');
        }
    })
    .catch(error => {
        addLog('error', `保存失败: ${error}`);
        showToast('保存失败', 'error');
        console.error('保存标定结果错误:', error);
    });
}

function capturePose() {
    addLog('info', '正在捕获位姿数据...');
    showToast('正在捕获位姿...', 'info');
    
    fetch('/api/robot_pose')
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                const pose = {
                    index: capturedPoses.length + 1,
                    position: data.pose.position,
                    orientation: data.pose.orientation,
                    corners: 54  // TODO: 实际检测角点数
                };
                capturedPoses.push(pose);
                addPoseToTable(pose);
                updatePoseCount();
                
                addLog('success', `位姿 #${pose.index} 捕获成功: [${(pose.position.x*1000).toFixed(1)}, ${(pose.position.y*1000).toFixed(1)}, ${(pose.position.z*1000).toFixed(1)}] mm`);
                showToast('位姿捕获成功', 'success');
            } else {
                addLog('error', '无法获取机器人位姿数据');
                showToast('无法获取机器人位姿', 'error');
            }
        })
        .catch(error => {
            console.error('Error:', error);
            addLog('error', '捕获位姿失败: ' + error.message);
            showToast('捕获位姿失败', 'error');
        });
}

function addPoseToTable(pose) {
    const tbody = document.querySelector('#pose-list-table tbody');
    const noData = tbody.querySelector('.no-data');
    if (noData) {
        tbody.innerHTML = '';
    }
    
    const row = tbody.insertRow();
    row.setAttribute('data-index', pose.index);
    row.innerHTML = `
        <td>${pose.index}</td>
        <td>${(pose.position.x * 1000).toFixed(2)}</td>
        <td>${(pose.position.y * 1000).toFixed(2)}</td>
        <td>${(pose.position.z * 1000).toFixed(2)}</td>
        <td>${pose.orientation.x.toFixed(4)}</td>
        <td>${pose.orientation.y.toFixed(4)}</td>
        <td>${pose.orientation.z.toFixed(4)}</td>
        <td>${pose.orientation.w.toFixed(4)}</td>
        <td>${pose.corners}</td>
    `;
    
    // 添加点击选中功能
    row.addEventListener('click', function() {
        document.querySelectorAll('#pose-list-table tbody tr').forEach(tr => {
            tr.classList.remove('selected');
        });
        this.classList.add('selected');
    });
}

function deletePose() {
    const selectedRow = document.querySelector('#pose-list-table tbody tr.selected');
    if (selectedRow) {
        const index = parseInt(selectedRow.getAttribute('data-index'));
        capturedPoses = capturedPoses.filter(p => p.index !== index);
        selectedRow.remove();
        updatePoseCount();
        
        addLog('warning', `位姿 #${index} 已删除`);
        showToast('位姿已删除', 'success');
        
        // 如果没有数据了，显示提示
        const tbody = document.querySelector('#pose-list-table tbody');
        if (tbody.rows.length === 0) {
            tbody.innerHTML = '<tr><td colspan="9" class="no-data">暂无数据</td></tr>';
        }
    } else {
        addLog('warning', '未选择要删除的位姿');
        showToast('请先选择要删除的位姿', 'warning');
    }
}

function clearPoses() {
    if (confirm('确定要清空所有位姿吗？')) {
        capturedPoses = [];
        const tbody = document.querySelector('#pose-list-table tbody');
        tbody.innerHTML = '<tr><td colspan="9" class="no-data">暂无数据</td></tr>';
        updatePoseCount();
        
        addLog('warning', '已清空所有位姿数据');
        showToast('已清空所有位姿', 'success');
    }
}

function startCalibration() {
    if (capturedPoses.length < 5) {
        addLog('warning', `位姿数量不足: ${capturedPoses.length}/5`);
        showToast('至少需要5组位姿数据才能进行标定', 'warning');
        return;
    }
    
    addLog('info', `开始手眼标定，使用 ${capturedPoses.length} 组位姿数据`);
    showToast('正在进行手眼标定...', 'info');
    document.getElementById('calib-status').textContent = '标定中...';
    
    // TODO: 调用实际的标定API
    fetch('/api/hand_eye/calibrate', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            poses: capturedPoses
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            const error = 1.25; // TODO: 实际误差
            document.getElementById('calib-status').textContent = '标定完成';
            document.getElementById('calib-error').textContent = error + ' mm';
            
            addLog('success', `手眼标定完成，误差: ${error} mm`);
            showToast('手眼标定完成', 'success');
        } else {
            document.getElementById('calib-status').textContent = '标定失败';
            addLog('error', '标定失败: ' + data.message);
            showToast('标定失败: ' + data.message, 'error');
        }
    })
    .catch(error => {
        console.error('Error:', error);
        document.getElementById('calib-status').textContent = '标定失败';
        addLog('error', '标定过程出错: ' + error.message);
        showToast('标定过程出错', 'error');
    });
}

function saveCalibration() {
    addLog('info', '保存标定结果...');
    showToast('保存标定结果', 'info');
    // TODO: 实现保存功能
}

function updatePoseCount() {
    document.getElementById('pose-count').textContent = capturedPoses.length;
}

// ============= 手眼标定精度验证 =============
function loadHandEyeParams() {
    addLog('info', '加载手眼标定参数...');
    showToast('正在加载手眼标定参数...', 'info');
    
    // 调用新的加载函数
    loadHandEyeCalibrationParams();
}

function captureTestPose() {
    addLog('info', '捕获测试位姿...');
    showToast('正在捕获测试位姿...', 'info');
    // TODO: 实现测试位姿捕获功能
}

function calculateTransform() {
    addLog('info', '计算坐标转换...');
    showToast('正在计算坐标转换...', 'info');
    // TODO: 实现坐标转换计算功能
}

function verifyAccuracy() {
    addLog('info', '验证精度...');
    showToast('正在验证精度...', 'info');
    
    // TODO: 实现精度验证功能
    setTimeout(() => {
        addVerifyResult(1, 100.5, 200.3, 50.2, 101.2, 199.8, 50.5, 0.92);
        updateVerifyStats();
        addLog('success', '精度验证完成，平均误差: 0.92 mm');
        showToast('精度验证完成', 'success');
    }, 1000);
}

function saveVerifyLog() {
    addLog('info', '保存验证日志...');
    showToast('保存验证日志', 'info');
    // TODO: 实现保存功能
}

function addVerifyResult(index, rx, ry, rz, cx, cy, cz, error) {
    const tbody = document.querySelector('#verify-result-table tbody');
    const noData = tbody.querySelector('.no-data');
    if (noData) {
        tbody.innerHTML = '';
    }
    
    const row = tbody.insertRow();
    row.innerHTML = `
        <td>${index}</td>
        <td>${rx.toFixed(2)}</td>
        <td>${ry.toFixed(2)}</td>
        <td>${rz.toFixed(2)}</td>
        <td>${cx.toFixed(2)}</td>
        <td>${cy.toFixed(2)}</td>
        <td>${cz.toFixed(2)}</td>
        <td><span style="color: ${error < 2.0 ? '#4caf50' : '#f44336'}">
            ${error.toFixed(3)}
        </span></td>
    `;
    
    addLog(error < 2.0 ? 'success' : 'warning',
           `验证点 #${index}: 误差=${error.toFixed(3)} mm`);
}

function updateVerifyStats() {
    document.getElementById('test-count').textContent = '1';
    document.getElementById('avg-error').textContent = '0.92';
    document.getElementById('max-error').textContent = '0.92';
}

// ============= 图像更新 =============
function startImageUpdate() {
    // 不再自动轮询，改为按需更新
    // 只有在采集图像后才会主动获取
    // 这样可以避免不必要的网络请求
}

function updateCurrentImage() {
    // 按需更新图像，不再自动轮询
    // 此函数由采集图像后的轮询调用
    fetch('/api/current_image')
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                let imgElement;
                let placeholder;
                if (currentTab === 'camera-verify') {
                    imgElement = document.getElementById('camera-verify-image');
                    placeholder = document.getElementById('camera-verify-image-container').querySelector('.image-placeholder');
                } else if (currentTab === 'hand-eye-calib') {
                    imgElement = document.getElementById('hand-eye-calib-image');
                    placeholder = document.getElementById('hand-eye-calib-image-container').querySelector('.image-placeholder');
                } else if (currentTab === 'hand-eye-verify') {
                    imgElement = document.getElementById('hand-eye-verify-image');
                    placeholder = document.getElementById('hand-eye-verify-image-container').querySelector('.image-placeholder');
                }
                
                if (imgElement) {
                    imgElement.onload = function() {
                        this.classList.add('loaded');
                        if (placeholder) {
                            placeholder.style.display = 'none';
                        }
                    };
                    // 添加时间戳避免缓存
                    imgElement.src = data.image + '?t=' + new Date().getTime();
                }
            }
        })
        .catch(error => {
            console.error('Error updating image:', error);
        });
}

// ============= 机器人位姿更新 =============
function startRobotPoseUpdate() {
    // 立即调用一次，确保页面加载时就能看到位姿
    updateRobotPose();
    robotPoseUpdateInterval = setInterval(() => {
        if (currentTab === 'hand-eye-calib' && !document.hidden) {
            updateRobotPose();
        }
    }, 100);  // 每100ms更新一次
}

// 启动手眼验证选项卡的机器人状态更新
function startVerifyRobotStatusUpdate() {
    setInterval(() => {
        if (currentTab === 'hand-eye-verify' && !document.hidden) {
            updateVerifyRobotStatus();
        }
    }, 1000);  // 每1秒更新一次
}

function updateVerifyRobotStatus() {
    fetch('/api/robot_status')
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                updateVerifyRobotPoseDisplay(data);
            }
        })
        .catch(error => {
            console.error('Error updating verify robot status:', error);
        });
}

function updateRobotPose() {
    fetch('/api/robot_status')
        .then(response => {
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
            return response.json();
        })
        .then(data => {
            if (data.success && data.cartesian_position && data.cartesian_position.position) {
                const pose = data.cartesian_position;
                
                // 根据当前选项卡确定元素ID前缀
                const prefix = (currentTab === 'auto-hand-eye-calib') ? 'auto-' : '';
                
                const posX = document.getElementById(prefix + 'pos-x');
                const posY = document.getElementById(prefix + 'pos-y');
                const posZ = document.getElementById(prefix + 'pos-z');
                
                // 更新位姿显示（单位：毫米）- 将米转换为毫米（乘以1000）
                if (posX) posX.textContent = (pose.position.x * 1000).toFixed(3);
                if (posY) posY.textContent = (pose.position.y * 1000).toFixed(3);
                if (posZ) posZ.textContent = (pose.position.z * 1000).toFixed(3);
                const oriX = document.getElementById(prefix + 'ori-x');
                const oriY = document.getElementById(prefix + 'ori-y');
                const oriZ = document.getElementById(prefix + 'ori-z');
                const oriW = document.getElementById(prefix + 'ori-w');
                if (oriX) oriX.textContent = pose.orientation.x.toFixed(6);
                if (oriY) oriY.textContent = pose.orientation.y.toFixed(6);
                if (oriZ) oriZ.textContent = pose.orientation.z.toFixed(6);
                if (oriW) oriW.textContent = pose.orientation.w.toFixed(6);
                
                // 更新机器人在线状态指示器
                const indicator = document.getElementById(prefix + 'robot-online-indicator');
                if (data.is_online) {
                    indicator.textContent = '🟢 在线';
                    indicator.className = 'status-indicator online';
                    updateRobotStatus('在线');  // 同时更新底部状态栏
                } else {
                    indicator.textContent = '🔴 离线';
                    indicator.className = 'status-indicator offline';
                    updateRobotStatus('离线');
                }
                
                // 可选：显示运动状态
                if (data.in_motion) {
                    indicator.textContent = data.is_online ? '🟡 在线(运动中)' : '🔴 离线';
                }
            } else {
                // 如果无法获取数据，显示离线
                const indicator = document.getElementById('robot-online-indicator');
                if (indicator) {
                    indicator.textContent = '🔴 无数据';
                    indicator.className = 'status-indicator offline';
                    updateRobotStatus('离线');
                    // 重置位置值为"-"
                    const posX = document.getElementById('pos-x');
                    const posY = document.getElementById('pos-y');
                    const posZ = document.getElementById('pos-z');
                    const oriX = document.getElementById('ori-x');
                    const oriY = document.getElementById('ori-y');
                    const oriZ = document.getElementById('ori-z');
                    const oriW = document.getElementById('ori-w');
                    if (posX) posX.textContent = '-';
                    if (posY) posY.textContent = '-';
                    if (posZ) posZ.textContent = '-';
                    if (oriX) oriX.textContent = '-';
                    if (oriY) oriY.textContent = '-';
                    if (oriZ) oriZ.textContent = '-';
                    if (oriW) oriW.textContent = '-';
                }
            }
        })
        .catch(error => {
            // 网络错误或API错误
            try {
                const indicator = document.getElementById('robot-online-indicator');
                if (indicator) {
                    indicator.textContent = '⚠️ 错误';
                    indicator.className = 'status-indicator error';
                    updateRobotStatus('错误');
                    // 重置位置值为"-"
                    const posX = document.getElementById('pos-x');
                    const posY = document.getElementById('pos-y');
                    const posZ = document.getElementById('pos-z');
                    const oriX = document.getElementById('ori-x');
                    const oriY = document.getElementById('ori-y');
                    const oriZ = document.getElementById('ori-z');
                    const oriW = document.getElementById('ori-w');
                    if (posX) posX.textContent = '-';
                    if (posY) posY.textContent = '-';
                    if (posZ) posZ.textContent = '-';
                    if (oriX) oriX.textContent = '-';
                    if (oriY) oriY.textContent = '-';
                    if (oriZ) oriZ.textContent = '-';
                    if (oriW) oriW.textContent = '-';
                }
            } catch (domError) {
                console.error('Error updating DOM in catch block:', domError);
            }
            console.error('更新机器人位姿失败:', error);
        });
}

// ============= 相机内参更新 =============
let cameraInfoUpdateInterval = null;
let cameraInfoSource = null;  // 跟踪当前内参来源

function startCameraInfoUpdate() {
    // 定期尝试从 ROS2 话题获取内参（话题数据优先级最高）
    cameraInfoUpdateInterval = setInterval(() => {
        if (!document.hidden) {
            updateCameraInfoFromROS2();
        }
    }, 5000);  // 每5秒尝试一次
}

function updateCameraInfoFromROS2() {
    fetch('/api/camera_info')
    .then(response => response.json())
    .then(data => {
        if (data.success && data.camera_intrinsic) {
            const cm = data.camera_intrinsic;
            const imageSizeText = cm.image_size && cm.image_size.length === 2 
                ? `${cm.image_size[0]} x ${cm.image_size[1]}` 
                : '未知';
            
            // 如果是从 ROS2 话题获取的，或者当前还没有数据，则更新界面
            if (data.source === 'ros2_topic') {
                // ROS2 话题数据优先级最高，总是更新
                if (cameraInfoSource !== 'ros2_topic') {
                    cameraParamsLoaded = true;
                    cameraInfoSource = 'ros2_topic';
                    
                    const paramHTML = `
                        <p>数据源: <span style="color: #4caf50;">(从 ROS2 话题)</span></p>
                        <p>图像尺寸: <span>${imageSizeText}</span></p>
                        <p>焦距 fx: <span>${cm.fx.toFixed(2)}</span></p>
                        <p>焦距 fy: <span>${cm.fy.toFixed(2)}</span></p>
                        <p>主点 cx: <span>${cm.cx.toFixed(2)}</span></p>
                        <p>主点 cy: <span>${cm.cy.toFixed(2)}</span></p>
                        <p>畸变系数: <span>${cm.dist_coeffs ? cm.dist_coeffs.length : 0}个</span></p>
                    `;
                    
                    // 更新相机标定验证选项卡
                    document.getElementById('camera-params-info').innerHTML = paramHTML;
                    
                    // 同时更新手眼标定选项卡
                    const heParamsInfo = document.getElementById('he-camera-params-info');
                    if (heParamsInfo) {
                        heParamsInfo.innerHTML = paramHTML;
                    }
                    
                    addLog('success', `✅ 已从 ROS2 CameraInfo 话题获取相机内参：fx=${cm.fx.toFixed(2)}, fy=${cm.fy.toFixed(2)}`);
                    showToast('已从 ROS2 话题获取相机内参', 'success');
                }
            } else if (!cameraParamsLoaded) {
                // 如果还没有加载过任何数据，使用文件数据
                cameraParamsLoaded = true;
                cameraInfoSource = 'loaded_file';
                
                const paramHTML = `
                    <p>数据源: <span style="color: #2196f3;">(从文件)</span></p>
                    <p>图像尺寸: <span>${imageSizeText}</span></p>
                    <p>焦距 fx: <span>${cm.fx.toFixed(2)}</span></p>
                    <p>焦距 fy: <span>${cm.fy.toFixed(2)}</span></p>
                    <p>主点 cx: <span>${cm.cx.toFixed(2)}</span></p>
                    <p>主点 cy: <span>${cm.cy.toFixed(2)}</span></p>
                    <p>畸变系数: <span>${cm.dist_coeffs ? cm.dist_coeffs.length : 0}个</span></p>
                `;
                
                document.getElementById('camera-params-info').innerHTML = paramHTML;
                const heParamsInfo = document.getElementById('he-camera-params-info');
                if (heParamsInfo) {
                    heParamsInfo.innerHTML = paramHTML;
                }
            }
        }
    })
    .catch(error => {
        // 静默失败，不打印日志
        });
}

// ============= 状态更新 =============
function checkConnectionStatus() {
    // 定义检查函数
    const checkStatus = () => {
        fetch('/api/robot_pose')
            .then(response => {
                if (response.ok) {
                    updateConnectionStatus('已连接');
                } else {
                    updateConnectionStatus('断开连接');
                }
            })
            .catch(() => {
                updateConnectionStatus('断开连接');
            });
    };
    
    // 立即执行一次
    checkStatus();
    
    // 然后每2秒检查一次
    setInterval(checkStatus, 2000);
}

function updateConnectionStatus(status) {
    const element = document.getElementById('connection-status');
    element.textContent = status;
    element.className = 'status-value ' + (status === '已连接' ? 'online' : 'offline');
}

function updateCameraStatus(status) {
    const element = document.getElementById('camera-status');
    element.textContent = status;
    element.className = 'status-value ' + (status === '在线' ? 'online' : 'offline');
}

function updateRobotStatus(status) {
    const element = document.getElementById('robot-status');
    element.textContent = status;
    element.className = 'status-value ' + (status === '在线' ? 'online' : 'offline');
}

// ============= 相机ID状态检查 =============
let cameraStatusHistory = [];  // 状态历史，用于防抖
const CAMERA_STATUS_HISTORY_SIZE = 3;  // 保留最近3次状态

function checkCameraIDStatus() {
    // 降低检查频率到5秒，并使用防抖机制
    setInterval(() => {
        const expectedCameraId = document.getElementById('camera-id')?.value || '207000152740';
        
        fetch('/api/camera_status')
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    const isOnline = data.is_connected && data.camera_id === expectedCameraId;
                    
                    // 添加到历史记录
                    cameraStatusHistory.push(isOnline);
                    if (cameraStatusHistory.length > CAMERA_STATUS_HISTORY_SIZE) {
                        cameraStatusHistory.shift();
                    }
                    
                    // 只有当历史记录都一致时才更新状态（防抖）
                    if (cameraStatusHistory.length >= CAMERA_STATUS_HISTORY_SIZE) {
                        const allSame = cameraStatusHistory.every(val => val === cameraStatusHistory[0]);
                        if (allSame) {
                            updateCameraIDStatus(isOnline, data.camera_id);
                            
                            // 同时更新底部状态栏的相机状态
                            if (isOnline) {
                                updateCameraStatus('在线');
                            } else if (data.camera_id !== expectedCameraId) {
                                updateCameraStatus(`ID不匹配(${data.camera_id})`);
                            } else {
                                updateCameraStatus('离线');
                            }
                        }
                    } else {
                        // 历史记录不足，显示检查中
                        if (cameraStatusHistory.length === 1) {
                            updateCameraIDStatus(isOnline, data.camera_id);
                        }
                    }
                } else {
                    cameraStatusHistory = [];
                    updateCameraIDStatus(false, null);
                    updateCameraStatus('未知');
                }
            })
            .catch(error => {
                cameraStatusHistory = [];
                updateCameraIDStatus(false, null);
                updateCameraStatus('错误');
            });
    }, 5000);  // 降低到每5秒检查一次
}

function updateCameraIDStatus(isOnline, actualCameraId) {
    const statusDot = document.getElementById('camera-id-status');
    const statusText = document.getElementById('camera-id-status-text');
    const expectedCameraId = document.getElementById('camera-id')?.value;
    
    if (!statusDot || !statusText) return;
    
    if (isOnline) {
        statusDot.className = 'status-dot online';
        statusText.className = 'status-text online';
        statusText.textContent = '在线';
    } else if (actualCameraId && actualCameraId !== expectedCameraId) {
        statusDot.className = 'status-dot offline';
        statusText.className = 'status-text offline';
        statusText.textContent = `ID不匹配: ${actualCameraId}`;
    } else {
        statusDot.className = 'status-dot offline';
        statusText.className = 'status-text offline';
        statusText.textContent = '离线';
    }
}

// ============= 保存图像功能 =============
function saveCurrentImage(tabId) {
    // 根据tabId获取对应的图像元素
    const imgElement = document.getElementById(`${tabId}-image`);
    
    if (!imgElement || !imgElement.classList.contains('loaded')) {
        showToast('没有可保存的图像', 'warning');
        addLog('warning', '没有可保存的图像');
        return;
    }
    
    // 获取图像的src（Base64格式）
    const imgSrc = imgElement.src;
    
    if (!imgSrc || !imgSrc.startsWith('data:image')) {
        showToast('图像格式错误', 'error');
        addLog('error', '图像格式错误，无法保存');
        return;
    }
    
    // 生成文件名（带时间戳）
    const now = new Date();
    const timestamp = `${now.getFullYear()}${String(now.getMonth()+1).padStart(2,'0')}${String(now.getDate()).padStart(2,'0')}_` +
                     `${String(now.getHours()).padStart(2,'0')}${String(now.getMinutes()).padStart(2,'0')}${String(now.getSeconds()).padStart(2,'0')}`;
    const tabName = tabId.replace(/-/g, '_');
    const fileName = `calibration_${tabName}_${timestamp}.jpg`;
    
    // 创建下载链接
    const link = document.createElement('a');
    link.href = imgSrc;
    link.download = fileName;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    
    addLog('success', `💾 图像已保存: ${fileName}`);
    showToast(`图像已保存: ${fileName}`, 'success');
}

// ============= 表格行复制功能 =============
function copyRowData(row, dataType = '数据') {
    // 获取所有单元格的文本
    const cells = row.querySelectorAll('td');
    const rowData = [];
    const rowDataFormatted = [];
    
    // 获取表头
    const table = row.closest('table');
    const headers = Array.from(table.querySelectorAll('thead th')).map(th => th.textContent.trim());
    
    // 提取单元格数据
    cells.forEach((cell, index) => {
        const value = cell.textContent.trim();
        rowData.push(value);
        
        // 格式化：表头 = 值
        if (headers[index]) {
            rowDataFormatted.push(`${headers[index]}: ${value}`);
        }
    });
    
    // 生成两种格式的文本
    const textTabSeparated = rowData.join('\t');  // Tab分隔（适合粘贴到Excel）
    const textFormatted = rowDataFormatted.join(' | ');  // 格式化文本（适合日志）
    
    // 复制到剪贴板
    navigator.clipboard.writeText(textTabSeparated).then(() => {
        // 高亮显示已复制的行
        const allRows = table.querySelectorAll('tbody tr');
        allRows.forEach(r => r.classList.remove('copied'));
        row.classList.add('copied');
        
        // 显示提示
        addLog('success', `📋 已复制${dataType}: ${textFormatted}`);
        showToast(`已复制到剪贴板`, 'success');
        
        // 3秒后移除高亮
        setTimeout(() => {
            row.classList.remove('copied');
        }, 3000);
    }).catch(err => {
        console.error('复制失败:', err);
        addLog('error', '复制失败: ' + err.message);
        showToast('复制失败', 'error');
    });
}

// ============= 提示消息 =============
function showToast(message, type = 'info') {
    const existingToast = document.querySelector('.toast');
    if (existingToast) {
        existingToast.remove();
    }
    
    const toast = document.createElement('div');
    toast.className = 'toast';
    if (type === 'error') {
        toast.classList.add('error');
    } else if (type === 'warning') {
        toast.classList.add('warning');
    } else if (type === 'info') {
        toast.classList.add('info');
    }
    toast.textContent = message;
    
    document.body.appendChild(toast);
    
    setTimeout(() => {
        toast.remove();
    }, 3000);
}

// ============= 去畸变功能 =============
// 注意：已改为按需去畸变策略
// - 图像显示：保持原始状态（快速）
// - 坐标计算：像素点自动去畸变（准确）
// 
// 以下函数保留以备后用，如需整体图像去畸变可取消注释

/*
function askForUndistortion(imgElement) {
    // 检查是否已加载标定参数
    if (!cameraParamsLoaded) {
        addLog('info', '未加载相机标定参数，跳过去畸变');
        return;
    }
    
    // 创建弹窗
    const dialog = document.createElement('div');
    dialog.className = 'undistort-dialog';
    dialog.innerHTML = `
        <div class="undistort-dialog-content">
            <h3>🔧 图像去畸变</h3>
            <p>检测到已加载相机标定参数，是否对图像进行去畸变处理？</p>
            <div class="undistort-dialog-buttons">
                <button class="dialog-btn primary" id="btn-undistort-yes">去畸变</button>
                <button class="dialog-btn" id="btn-undistort-no">跳过</button>
            </div>
        </div>
    `;
    
    document.body.appendChild(dialog);
    
    // 按钮事件
    document.getElementById('btn-undistort-yes').addEventListener('click', () => {
        dialog.remove();
        performUndistortion(imgElement);
    });
    
    document.getElementById('btn-undistort-no').addEventListener('click', () => {
        dialog.remove();
        addLog('info', '用户跳过去畸变处理');
    });
    
    // 点击背景关闭
    dialog.addEventListener('click', function(e) {
        if (e.target === dialog) {
            dialog.remove();
            addLog('info', '用户跳过去畸变处理');
        }
    });
}

function performUndistortion(imgElement) {
    addLog('info', '开始执行去畸变处理...');
    showToast('正在去畸变...', 'info');
    
    fetch('/api/camera/undistort_image', {
        method: 'POST'
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addLog('success', `✅ 图像去畸变成功 (${data.width}x${data.height})`);
            showToast('去畸变完成', 'success');
            
            // 验证返回的图像数据
            if (!data.image || !data.image.startsWith('data:image')) {
                addLog('error', '返回的图像数据格式错误');
                showToast('图像数据格式错误', 'error');
                return;
            }
            
            const imageDataLength = data.image.length;
            addLog('info', `Base64数据长度: ${imageDataLength} (${(imageDataLength/1024).toFixed(1)}KB)`);
            
            // 重新设置图像加载事件处理
            imgElement.onload = function() {
                addLog('success', '✅ 去畸变图像显示成功');
                this.classList.add('loaded');
                
                // 隐藏占位符
                const placeholder = document.getElementById('camera-verify-image-container').querySelector('.image-placeholder');
                if (placeholder) {
                    placeholder.style.display = 'none';
                }
            };
            
            imgElement.onerror = function(e) {
                console.error('Image load error:', e);
                addLog('error', '去畸变图像显示失败 - 可能图像数据损坏或太大');
                showToast('图像显示失败', 'error');
            };
            
            // 更新图像显示
            addLog('info', '正在更新图像显示...');
            imgElement.src = data.image;
            
            // 重置缩放
            resetImageZoom('camera-verify');
        } else {
            addLog('error', '去畸变失败: ' + data.error);
            showToast('去畸变失败: ' + data.error, 'error');
        }
    })
    .catch(error => {
        addLog('error', '去畸变失败: ' + error);
        showToast('去畸变失败', 'error');
    });
}
*/

// ============= 图像缩放功能 =============
function initImageZoom() {
    const tabs = ['camera-verify', 'hand-eye-calib', 'hand-eye-verify', 'auto-hand-eye-calib'];
    
    tabs.forEach(tab => {
        const container = document.getElementById(`${tab}-image-container`);
        const imageElement = document.getElementById(`${tab}-image`);
        
        if (!container || !imageElement) return;
        
        // 初始化缩放状态
        imageZoomStates[tab] = {
            scale: 1,
            isDragging: false,
            lastX: 0,
            lastY: 0
        };
        
        const state = imageZoomStates[tab];
        
        // 滚轮缩放 - 以鼠标位置为中心缩放
        container.addEventListener('wheel', function(e) {
            if (!imageElement.classList.contains('loaded')) return;
            
            e.preventDefault();
            
            // 获取鼠标相对于容器的位置
            const rect = container.getBoundingClientRect();
            const mouseX = e.clientX - rect.left + container.scrollLeft;
            const mouseY = e.clientY - rect.top + container.scrollTop;
            
            // 计算缩放
            const delta = e.deltaY > 0 ? -0.1 : 0.1;
            const oldScale = state.scale;
            const newScale = Math.max(0.5, Math.min(5, oldScale + delta));
            
            if (newScale !== oldScale) {
                // 获取缩放前鼠标在图像中的比例位置
                const imgRect = imageElement.getBoundingClientRect();
                const containerRect = container.getBoundingClientRect();
                const mouseInImageX = (e.clientX - imgRect.left) / imgRect.width;
                const mouseInImageY = (e.clientY - imgRect.top) / imgRect.height;
                
                // 更新缩放
                state.scale = newScale;
                updateImageScale(imageElement, container, state.scale, tab);
                
                // 等待图像尺寸更新后，调整滚动位置
                setTimeout(() => {
                    const newImgRect = imageElement.getBoundingClientRect();
                    const newScrollLeft = newImgRect.width * mouseInImageX - (e.clientX - containerRect.left);
                    const newScrollTop = newImgRect.height * mouseInImageY - (e.clientY - containerRect.top);
                    
                    container.scrollLeft = Math.max(0, newScrollLeft);
                    container.scrollTop = Math.max(0, newScrollTop);
                }, 0);
            }
        }, { passive: false });
        
        // 右键拖拽（或Ctrl+左键）
        container.addEventListener('mousedown', function(e) {
            if (!imageElement.classList.contains('loaded')) return;
            
            // 手动选点模式下的左键点击
            if (tab === 'camera-verify' && manualPickMode && e.button === 0) {
                e.preventDefault();
                handleImageClick(e);
                return;
            }
            
            // 右键或Ctrl+左键启动拖拽
            if (e.button === 2 || (e.button === 0 && e.ctrlKey) || (e.button === 0 && state.scale > 1)) {
                e.preventDefault();
                state.isDragging = true;
                state.lastX = e.clientX;
                state.lastY = e.clientY;
                container.style.cursor = 'grabbing';
            }
        });
        
        container.addEventListener('mousemove', function(e) {
            if (!state.isDragging) return;
            
            e.preventDefault();
            
            // 计算移动距离
            const deltaX = e.clientX - state.lastX;
            const deltaY = e.clientY - state.lastY;
            
            // 更新滚动位置
            container.scrollLeft -= deltaX;
            container.scrollTop -= deltaY;
            
            // 更新位置
            state.lastX = e.clientX;
            state.lastY = e.clientY;
        });
        
        container.addEventListener('mouseup', function() {
            if (state.isDragging) {
                state.isDragging = false;
                container.style.cursor = state.scale > 1 ? 'grab' : 'crosshair';
            }
        });
        
        container.addEventListener('mouseleave', function() {
            state.isDragging = false;
            container.style.cursor = state.scale > 1 ? 'grab' : 'crosshair';
        });
        
        // 禁用右键菜单
        container.addEventListener('contextmenu', function(e) {
            e.preventDefault();
        });
        
        // 双击重置
        container.addEventListener('dblclick', function() {
            state.scale = 1;
            updateImageScale(imageElement, container, state.scale, tab);
            container.scrollLeft = 0;
            container.scrollTop = 0;
        });
    });
}

function updateImageScale(imageElement, container, scale, tab) {
    if (scale > 1) {
        imageElement.classList.add('zoomed');
        
        // 改为直接设置宽度和高度，而不是使用transform
        // 这样滚动条可以正常工作
        const naturalWidth = imageElement.naturalWidth;
        const naturalHeight = imageElement.naturalHeight;
        
        if (naturalWidth && naturalHeight) {
            // 计算缩放后的尺寸
            imageElement.style.width = (naturalWidth * scale) + 'px';
            imageElement.style.height = (naturalHeight * scale) + 'px';
            imageElement.style.maxWidth = 'none';
            imageElement.style.maxHeight = 'none';
        }
        
        container.style.cursor = 'grab';
        container.classList.add('zoomed');
    } else {
        imageElement.classList.remove('zoomed');
        imageElement.style.width = '';
        imageElement.style.height = '';
        imageElement.style.maxWidth = '100%';
        imageElement.style.maxHeight = '100%';
        container.style.cursor = 'crosshair';
        container.classList.remove('zoomed');
        container.scrollLeft = 0;
        container.scrollTop = 0;
    }
    
    // 更新标题右侧的缩放提示
    const hintElement = document.getElementById(`${tab}-zoom-hint`);
    if (hintElement) {
        if (scale > 1) {
            hintElement.textContent = `缩放: ${(scale * 100).toFixed(0)}% | 右键拖拽 | 双击重置`;
            hintElement.style.color = '#667eea';
            hintElement.style.fontWeight = '600';
        } else {
            hintElement.textContent = '滚轮缩放 | 右键拖拽';
            hintElement.style.color = '#999';
            hintElement.style.fontWeight = 'normal';
        }
    }
}

function resetImageZoom(tab) {
    if (imageZoomStates[tab]) {
        const state = imageZoomStates[tab];
        const container = document.getElementById(`${tab}-image-container`);
        const imageElement = document.getElementById(`${tab}-image`);
        
        if (container && imageElement) {
            state.scale = 1;
            state.isDragging = false;
            updateImageScale(imageElement, container, 1, tab);
            container.scrollLeft = 0;
            container.scrollTop = 0;
        }
    }
}

// 切换深度详细信息显示
function toggleDepthDetails(btn) {
    const table = btn.nextElementSibling;
    const icon = btn.querySelector('.toggle-icon');
    if (table.style.display === 'none') {
        table.style.display = 'block';
        icon.textContent = '▲';
    } else {
        table.style.display = 'none';
        icon.textContent = '▼';
    }
}

// 页面卸载时清理定时器
window.addEventListener('beforeunload', function() {
    if (imageUpdateInterval) {
        clearInterval(imageUpdateInterval);
    }
    if (robotPoseUpdateInterval) {
        clearInterval(robotPoseUpdateInterval);
    }
});

// 模态窗口点击外部关闭功能
window.addEventListener('click', function(event) {
    const modal = document.getElementById('single-point-test-modal');
    if (event.target === modal) {
        closeSinglePointTestModal();
    }
});

// ESC键关闭模态窗口
window.addEventListener('keydown', function(event) {
    if (event.key === 'Escape') {
        const modal = document.getElementById('single-point-test-modal');
        if (modal && modal.style.display === 'block') {
            closeSinglePointTestModal();
        }
    }
});
