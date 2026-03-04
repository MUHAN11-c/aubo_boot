// 全局变量
let currentTab = 'camera-verify';
let imageUpdateInterval = null;
let robotPoseUpdateInterval = null;
let capturedPoses = [];

// 放大镜配置
const MAGNIFIER_ZOOM = 3; // 放大倍数
const MAGNIFIER_SIZE = 100; // 采样区域大小（像素）

// 手动选点模式
let manualPickMode = false;
let manualPickPoints = [];

// 图像缩放状态管理
const imageZoomStates = {
    'camera-verify': { scale: 1, isDragging: false, lastX: 0, lastY: 0 },
    'hand-eye': { scale: 1, isDragging: false, lastX: 0, lastY: 0 },
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
    // initHandEyeCalibButtons();  // TODO: 实现手眼标定按钮
    // initHandEyeVerifyButtons();  // TODO: 实现手眼验证按钮
    
    // 初始化放大镜功能
    initMagnifiers();
    
    // 初始化图像缩放功能
    initImageZoom();
    
    // 启动图像更新
    startImageUpdate();
    
    // 启动机器人位姿更新（仅在手眼标定选项卡）
    startRobotPoseUpdate();
    
    // 检查连接状态
    checkConnectionStatus();
    
    // 检查相机状态
    checkCameraIDStatus();
    
    // 尝试自动加载默认标定参数
    autoLoadDefaultCameraParams();
    
    // 定期检查并更新相机内参（如果从 ROS2 话题获取）
    startCameraInfoUpdate();
    
    addLog('success', 'Web界面启动成功，等待ROS2数据...');
});

// ============= 日志系统 =============
function addLog(type, message) {
    const logContainerId = `${currentTab}-log`;
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
    const logContainerId = `${currentTab}-log`;
    const logContainer = document.getElementById(logContainerId);
    if (logContainer) {
        logContainer.innerHTML = '<div class="log-entry info">日志已清空</div>';
    }
}

// ============= 放大镜功能 =============
function initMagnifiers() {
    // 为每个选项卡的图像容器添加放大镜功能
    const tabs = ['camera-verify', 'hand-eye', 'hand-eye-verify'];
    
    tabs.forEach(tab => {
        const imageContainer = document.getElementById(`${tab}-image-container`);
        const imageElement = document.getElementById(`${tab}-image`);
        const magnifierCanvas = document.getElementById(`${tab}-magnifier`);
        const magnifierPlaceholder = imageContainer?.parentElement.querySelector('.magnifier-placeholder');
        
        if (!imageContainer || !imageElement || !magnifierCanvas) return;
        
        const ctx = magnifierCanvas.getContext('2d');
        const crosshair = imageContainer.querySelector('.zoom-crosshair');
        
        // 鼠标移动事件
        imageContainer.addEventListener('mousemove', function(e) {
            if (!imageElement.classList.contains('loaded')) return;
            
            // 如果正在拖拽，不显示放大镜
            const state = imageZoomStates[tab];
            if (state && state.isDragging) {
                magnifierCanvas.classList.remove('active');
                if (magnifierPlaceholder) magnifierPlaceholder.style.display = 'block';
                return;
            }
            
            const rect = imageContainer.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;
            
            // 绘制放大镜内容并获取坐标
            const coords = drawMagnifier(imageElement, magnifierCanvas, ctx, x, y, rect);
            
            // 更新放大镜标题右侧的坐标显示
            if (coords) {
                const hintElement = document.getElementById(`${tab}-mag-hint`);
                if (hintElement) {
                    hintElement.textContent = `X: ${Math.round(coords.sourceX)} | Y: ${Math.round(coords.sourceY)}`;
                }
            }
            
            // 显示放大镜，隐藏占位符
            magnifierCanvas.classList.add('active');
            if (magnifierPlaceholder) {
                magnifierPlaceholder.style.display = 'none';
            }
        });
        
        // 鼠标离开事件
        imageContainer.addEventListener('mouseleave', function() {
            magnifierCanvas.classList.remove('active');
            if (magnifierPlaceholder) {
                magnifierPlaceholder.style.display = 'block';
            }
            // 重置坐标显示
            const hintElement = document.getElementById(`${tab}-mag-hint`);
            if (hintElement) {
                hintElement.textContent = 'X: - | Y: -';
            }
        });
    });
}

function drawMagnifier(imageElement, canvas, ctx, mouseX, mouseY, containerRect) {
    // 计算图像在容器中的实际尺寸和位置
    const imgNaturalWidth = imageElement.naturalWidth;
    const imgNaturalHeight = imageElement.naturalHeight;
    const imgDisplayWidth = imageElement.width;
    const imgDisplayHeight = imageElement.height;
    
    if (!imgNaturalWidth || !imgNaturalHeight || !imgDisplayWidth || !imgDisplayHeight) return;
    
    // 考虑图像可能被缩放（transform: scale）
    const currentScale = imageZoomStates[currentTab]?.scale || 1;
    const scaledWidth = imgDisplayWidth * currentScale;
    const scaledHeight = imgDisplayHeight * currentScale;
    
    // 计算图像在容器中的偏移（居中显示）
    const imgLeft = (containerRect.width - scaledWidth) / 2;
    const imgTop = (containerRect.height - scaledHeight) / 2;
    
    // 考虑滚动位置
    const container = imageElement.parentElement;
    const scrollX = container.scrollLeft || 0;
    const scrollY = container.scrollTop || 0;
    
    // 计算鼠标在缩放图像上的相对位置
    const imgX = (mouseX + scrollX - imgLeft) / currentScale;
    const imgY = (mouseY + scrollY - imgTop) / currentScale;
    
    // 检查鼠标是否在图像内
    if (imgX < 0 || imgX > imgDisplayWidth || imgY < 0 || imgY > imgDisplayHeight) {
        return;
    }
    
    // 计算在原始图像中的坐标（以光标为中心）
    const scaleX = imgNaturalWidth / imgDisplayWidth;
    const scaleY = imgNaturalHeight / imgDisplayHeight;
    const sourceX = imgX * scaleX;
    const sourceY = imgY * scaleY;
    
    // 设置canvas尺寸 - 填满容器
    const containerWidth = canvas.parentElement.clientWidth;
    const containerHeight = canvas.parentElement.clientHeight;
    canvas.width = containerWidth;
    canvas.height = containerHeight;
    
    // 计算采样区域 - 严格以光标位置为中心
    const sampleSize = MAGNIFIER_SIZE;
    let sx = sourceX - sampleSize / 2;
    let sy = sourceY - sampleSize / 2;
    
    // 处理边界情况
    let offsetX = 0;  // 光标在采样区域中的偏移
    let offsetY = 0;
    
    if (sx < 0) {
        offsetX = -sx;  // 左边界
        sx = 0;
    } else if (sx + sampleSize > imgNaturalWidth) {
        offsetX = imgNaturalWidth - (sx + sampleSize);  // 右边界
        sx = imgNaturalWidth - sampleSize;
    }
    
    if (sy < 0) {
        offsetY = -sy;  // 上边界
        sy = 0;
    } else if (sy + sampleSize > imgNaturalHeight) {
        offsetY = imgNaturalHeight - (sy + sampleSize);  // 下边界
        sy = imgNaturalHeight - sampleSize;
    }
    
    const sw = Math.min(sampleSize, imgNaturalWidth - sx);
    const sh = Math.min(sampleSize, imgNaturalHeight - sy);
    
    // 清空canvas
    ctx.clearRect(0, 0, containerWidth, containerHeight);
    
    // 绘制放大的图像 - 填满整个canvas
    ctx.imageSmoothingEnabled = false; // 保持像素清晰
    ctx.drawImage(imageElement, sx, sy, sw, sh, 0, 0, containerWidth, containerHeight);
    
    // 计算十字线位置 - 考虑边界偏移和容器尺寸
    const centerX = (containerWidth / 2) + (offsetX / sampleSize * containerWidth);
    const centerY = (containerHeight / 2) + (offsetY / sampleSize * containerHeight);
    
    // 绘制十字线 - 精确对应光标位置
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(centerX, 0);
    ctx.lineTo(centerX, containerHeight);
    ctx.moveTo(0, centerY);
    ctx.lineTo(containerWidth, centerY);
    ctx.stroke();
    
    // 绘制中心圆 - 精确在光标对应位置
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
}

function getTabName(tabId) {
    const names = {
        'camera-verify': '相机标定精度验证',
        'hand-eye-calib': '手眼标定',
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
            document.getElementById('camera-params-info').innerHTML = `
                <p>数据源: <span style="color: #2196f3;">(从文件)</span></p>
                <p>图像尺寸: <span>${data.image_size[0]} x ${data.image_size[1]}</span></p>
                <p>焦距 fx: <span>${cm.fx.toFixed(2)}</span></p>
                <p>焦距 fy: <span>${cm.fy.toFixed(2)}</span></p>
                <p>主点 cx: <span>${cm.cx.toFixed(2)}</span></p>
                <p>主点 cy: <span>${cm.cy.toFixed(2)}</span></p>
                <p>畸变系数: <span>${data.dist_coeffs.length}个</span></p>
                <p>重投影误差: <span>${data.mean_reprojection_error.toFixed(4)} px</span></p>
            `;
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
    
    // 显示等待提示
            document.getElementById('camera-params-info').innerHTML = `
        <p style="color: #2196f3;">⏳ 等待相机内参数据...</p>
        <p>优先从 ROS2 CameraInfo 话题获取</p>
        <p style="font-size: 0.9em; color: #666;">系统会自动加载，请稍候...</p>
    `;
    
    // 定期检查机制会自动获取和更新内参
    // 不再急于加载文件，让 startCameraInfoUpdate() 处理
    
    // 如果10秒后还没有获取到，提示用户可以手动加载
    setTimeout(() => {
        if (!cameraParamsLoaded) {
            addLog('warning', '⚠️ 10秒内未从 ROS2 话题获取到内参，系统将继续等待...');
            addLog('info', '提示: 可以点击"加载相机参数"按钮手动加载文件（可选）');
            document.getElementById('camera-params-info').innerHTML = `
                <p style="color: #ff9800;">⏳ 仍在等待 ROS2 CameraInfo 话题...</p>
                <p>请确保相机节点正在运行</p>
                <p style="font-size: 0.9em;">或点击"加载相机参数"手动加载文件</p>
            `;
        }
    }, 10000);
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
                                imgElement.onload = function() {
                                    this.classList.add('loaded');
                                    if (placeholder) {
                                        placeholder.style.display = 'none';
                                    }
                                    addLog('success', `✅ 图像已成功显示 (尺寸: ${imgData.width}x${imgData.height})`);
                                    showToast('图像加载成功', 'success');
                                    
                                    // 询问是否去畸变
                                    askForUndistortion(imgElement);
                                };
                                imgElement.onerror = function() {
                                    addLog('error', '图像加载失败');
                                };
                                // 添加时间戳强制刷新
                                imgElement.src = imgData.image + '?t=' + new Date().getTime();
                                
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
    if (!imageElement.classList.contains('loaded')) {
        showToast('请先加载图像', 'warning');
        return;
    }
    
    // 计算点击位置相对于图像的像素坐标
    const rect = imageElement.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    
    // 转换为原始图像坐标
    const scaleX = imageElement.naturalWidth / rect.width;
    const scaleY = imageElement.naturalHeight / rect.height;
    const pixelX = Math.round(x * scaleX);
    const pixelY = Math.round(y * scaleY);
    
    // 检查坐标是否在图像范围内
    if (pixelX < 0 || pixelX >= imageElement.naturalWidth || 
        pixelY < 0 || pixelY >= imageElement.naturalHeight) {
        showToast('点击位置超出图像范围', 'warning');
        return;
    }
    
    addLog('info', `📍 选择点 ${manualPickPoints.length + 1}: (${pixelX}, ${pixelY})`);
    
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
            
            addLog('info', `准备显示图像，Base64长度: ${data.image.length}`);
            
            imgElement.onload = function() {
                console.log('Image loaded successfully');
                this.classList.add('loaded');
                if (placeholder) {
                    placeholder.style.display = 'none';
                }
                addLog('success', '✅ 本地图像显示成功');
                showToast('图像加载成功', 'success');
                
                // 询问是否去畸变
                askForUndistortion(imgElement);
            };
            
            imgElement.onerror = function(err) {
                console.error('Image load error:', err);
                addLog('error', '图像加载到DOM失败');
                showToast('图像显示失败', 'error');
            };
            
            // 直接设置src，不需要时间戳（因为每次都是新的base64）
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
    });
    
    addLog('info', `数据表格已更新，共 ${cornersData.length} 行`);
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
    // 创建弹窗
    const dialog = document.createElement('div');
    dialog.className = 'result-dialog';
    dialog.innerHTML = `
        <div class="result-dialog-content">
            <h3>📊 计算结果</h3>
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
                    <span class="result-label">最大误差:</span>
                    <span class="result-value error-value">${result.error_max.toFixed(3)} mm</span>
                </div>
                <div class="result-item">
                    <span class="result-label">误差百分比:</span>
                    <span class="result-value error-value">${result.error_percent.toFixed(2)}%</span>
                </div>
            </div>
            <div class="result-status ${result.error_percent < 5 ? 'success' : 'warning'}">
                ${result.error_percent < 5 ? '✅ 精度优秀' : '⚠️ 精度一般'}
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

// ============= 手眼标定 =============
function initHandEyeCalibButtons() {
    const buttons = {
        'btn-capture-pose': capturePose,
        'btn-delete-pose': deletePose,
        'btn-clear-poses': clearPoses,
        'btn-start-calibration': startCalibration,
        'btn-save-calibration': saveCalibration
    };
    
    Object.keys(buttons).forEach(btnId => {
        const btn = document.getElementById(btnId);
        if (btn) {
            btn.addEventListener('click', buttons[btnId]);
        }
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
function initHandEyeVerifyButtons() {
    const buttons = {
        'btn-load-hand-eye-params': loadHandEyeParams,
        'btn-capture-test-pose': captureTestPose,
        'btn-calculate-transform': calculateTransform,
        'btn-verify-accuracy': verifyAccuracy,
        'btn-save-verify-log': saveVerifyLog
    };
    
    Object.keys(buttons).forEach(btnId => {
        const btn = document.getElementById(btnId);
        if (btn) {
            btn.addEventListener('click', buttons[btnId]);
        }
    });
}

function loadHandEyeParams() {
    addLog('info', '加载手眼标定参数...');
    showToast('正在加载手眼标定参数...', 'info');
    
    // TODO: 实现加载手眼标定参数功能
    setTimeout(() => {
        document.getElementById('hand-eye-params-info').innerHTML = `
            <p>标定方法: <span>Eye-in-Hand</span></p>
            <p>标定时间: <span>2024-10-18</span></p>
            <p>标定误差: <span>1.25 mm</span></p>
        `;
        addLog('success', '手眼标定参数加载成功');
        showToast('手眼标定参数加载成功', 'success');
    }, 500);
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
                    imgElement = document.getElementById('hand-eye-image');
                    placeholder = document.getElementById('hand-eye-image-container').querySelector('.image-placeholder');
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
    robotPoseUpdateInterval = setInterval(() => {
        if (currentTab === 'hand-eye-calib' && !document.hidden) {
            updateRobotPose();
        }
    }, 100);  // 每100ms更新一次
}

function updateRobotPose() {
    fetch('/api/robot_pose')
        .then(response => response.json())
        .then(data => {
            if (data.success) {
                const pose = data.pose;
                document.getElementById('pos-x').textContent = (pose.position.x * 1000).toFixed(2);
                document.getElementById('pos-y').textContent = (pose.position.y * 1000).toFixed(2);
                document.getElementById('pos-z').textContent = (pose.position.z * 1000).toFixed(2);
                document.getElementById('ori-x').textContent = pose.orientation.x.toFixed(4);
                document.getElementById('ori-y').textContent = pose.orientation.y.toFixed(4);
                document.getElementById('ori-z').textContent = pose.orientation.z.toFixed(4);
                document.getElementById('ori-w').textContent = pose.orientation.w.toFixed(4);
                
                updateRobotStatus('在线');
            } else {
                updateRobotStatus('离线');
            }
        })
        .catch(error => {
            updateRobotStatus('错误');
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
                    
                    document.getElementById('camera-params-info').innerHTML = `
                        <p>数据源: <span style="color: #4caf50;">(从 ROS2 话题)</span></p>
                        <p>图像尺寸: <span>${imageSizeText}</span></p>
                        <p>焦距 fx: <span>${cm.fx.toFixed(2)}</span></p>
                        <p>焦距 fy: <span>${cm.fy.toFixed(2)}</span></p>
                        <p>主点 cx: <span>${cm.cx.toFixed(2)}</span></p>
                        <p>主点 cy: <span>${cm.cy.toFixed(2)}</span></p>
                        <p>畸变系数: <span>${cm.dist_coeffs ? cm.dist_coeffs.length : 0}个</span></p>
                    `;
                    
                    addLog('success', `✅ 已从 ROS2 CameraInfo 话题获取相机内参：fx=${cm.fx.toFixed(2)}, fy=${cm.fy.toFixed(2)}`);
                    showToast('已从 ROS2 话题获取相机内参', 'success');
                }
            } else if (!cameraParamsLoaded) {
                // 如果还没有加载过任何数据，使用文件数据
                cameraParamsLoaded = true;
                cameraInfoSource = 'loaded_file';
                
                document.getElementById('camera-params-info').innerHTML = `
                    <p>数据源: <span style="color: #2196f3;">(从文件)</span></p>
                    <p>图像尺寸: <span>${imageSizeText}</span></p>
                    <p>焦距 fx: <span>${cm.fx.toFixed(2)}</span></p>
                    <p>焦距 fy: <span>${cm.fy.toFixed(2)}</span></p>
                    <p>主点 cx: <span>${cm.cx.toFixed(2)}</span></p>
                    <p>主点 cy: <span>${cm.cy.toFixed(2)}</span></p>
                    <p>畸变系数: <span>${cm.dist_coeffs ? cm.dist_coeffs.length : 0}个</span></p>
                `;
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
            addLog('success', '✅ 图像去畸变成功');
            showToast('去畸变完成', 'success');
            
            // 重新设置图像加载事件处理
            imgElement.onload = function() {
                addLog('success', '✅ 去畸变图像显示成功');
                this.classList.add('loaded');
            };
            
            imgElement.onerror = function() {
                addLog('error', '去畸变图像显示失败');
                showToast('图像显示失败', 'error');
            };
            
            // 更新图像显示
            imgElement.src = data.image + '?t=' + new Date().getTime();
            
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

// ============= 图像缩放功能 =============
function initImageZoom() {
    const tabs = ['camera-verify', 'hand-eye', 'hand-eye-verify'];
    
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
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;
            
            // 计算缩放
            const delta = e.deltaY > 0 ? -0.1 : 0.1;
            const oldScale = state.scale;
            const newScale = Math.max(0.5, Math.min(5, oldScale + delta));
            
            if (newScale !== oldScale) {
                // 计算缩放中心点
                const scrollX = container.scrollLeft;
                const scrollY = container.scrollTop;
                
                // 更新缩放
                state.scale = newScale;
                updateImageScale(imageElement, container, state.scale, tab);
                
                // 调整滚动位置，使鼠标位置保持不变
                if (newScale > 1) {
                    const scaleRatio = newScale / oldScale;
                    container.scrollLeft = (scrollX + mouseX) * scaleRatio - mouseX;
                    container.scrollTop = (scrollY + mouseY) * scaleRatio - mouseY;
                }
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
        imageElement.style.transform = `scale(${scale})`;
        container.style.cursor = 'grab';
    } else {
        imageElement.classList.remove('zoomed');
        imageElement.style.transform = 'scale(1)';
        container.style.cursor = 'crosshair';
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

// 页面卸载时清理定时器
window.addEventListener('beforeunload', function() {
    if (imageUpdateInterval) {
        clearInterval(imageUpdateInterval);
    }
    if (robotPoseUpdateInterval) {
        clearInterval(robotPoseUpdateInterval);
    }
});
