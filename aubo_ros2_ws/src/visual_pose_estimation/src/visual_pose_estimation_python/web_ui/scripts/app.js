// API服务器配置
const API_BASE_URL = 'http://localhost:8088';

// 根据图像尺寸优化显示容器
function optimizeImageDisplay(img, container, options = {}) {
    // 默认选项
    const opts = {
        maxWidth: options.maxWidth || null,      // 最大宽度（像素），null表示使用容器宽度
        maxHeight: options.maxHeight || null,    // 最大高度（像素），null表示使用容器高度
        minAspectRatio: options.minAspectRatio || 0.5,  // 最小宽高比
        maxAspectRatio: options.maxAspectRatio || 2.0,   // 最大宽高比
        padding: options.padding || 0,          // 容器内边距（默认0，让图像占满）
        maintainAspectRatio: options.maintainAspectRatio !== false,  // 是否保持宽高比
        fillContainer: options.fillContainer !== false  // 是否占满容器（默认true）
    };
    
    // 等待图像加载完成
    if (img.complete && img.naturalWidth > 0) {
        applyImageOptimization(img, container, opts);
    } else {
        img.onload = function() {
            applyImageOptimization(img, container, opts);
        };
        img.onerror = function() {
            console.error('图像加载失败');
        };
    }
}

// 应用图像显示优化
function applyImageOptimization(img, container, options) {
    const imgWidth = img.naturalWidth;
    const imgHeight = img.naturalHeight;
    
    // 如果图像尺寸无效，使用默认尺寸640x480
    const actualWidth = imgWidth > 0 ? imgWidth : 640;
    const actualHeight = imgHeight > 0 ? imgHeight : 480;
    
    if (imgWidth === 0 || imgHeight === 0) {
        console.warn('图像尺寸无效，使用默认尺寸640x480');
        // 更新占位符显示默认尺寸
        const placeholderSubtext = container.querySelector ? container.querySelector('.placeholder-subtext') : null;
        if (placeholderSubtext) {
            placeholderSubtext.textContent = `640 × 480 (默认)`;
        }
    }
    
    const imgAspectRatio = actualWidth / actualHeight;
    
    // 获取容器的父元素（实际显示区域）
    let displayContainer = container;
    if (container.classList && container.classList.contains('image-placeholder')) {
        displayContainer = container.closest('.main-image-container') || 
                          container.closest('.debug-image-content') ||
                          container.parentElement;
    } else if (container.id && (container.id.includes('image-content') || container.id.includes('main-image') || container.id.includes('template-image'))) {
        // 如果容器本身就是显示容器
        displayContainer = container;
    } else {
        // 尝试查找父容器
        displayContainer = container.parentElement || container;
    }
    
    if (!displayContainer) {
        console.warn('无法找到显示容器');
        return;
    }
    
    // 获取容器的可用空间（使用 offsetWidth/offsetHeight 更准确）
    // 默认尺寸：640x480 (4:3 宽高比)
    const containerWidth = displayContainer.offsetWidth || displayContainer.clientWidth || 640;
    const containerHeight = displayContainer.offsetHeight || displayContainer.clientHeight || 480;
    
    // 如果要求占满容器
    if (options.fillContainer) {
        // 直接设置图像占满容器（忽略padding）
        img.style.width = '100%';
        img.style.height = '100%';
        img.style.objectFit = 'cover';  // 使用cover以占满容器，可能会裁剪部分图像
        img.style.display = 'block';
        img.style.margin = '0';
        img.style.padding = '0';
        
        // 确保容器没有内边距
        if (container.classList && container.classList.contains('image-placeholder')) {
            container.style.padding = '0';
            container.style.display = 'block';
            container.style.margin = '0';
        }
    } else {
        const availableWidth = options.maxWidth || (containerWidth - options.padding * 2);
        const availableHeight = options.maxHeight || (containerHeight - options.padding * 2);
        // 计算最佳显示尺寸（保持宽高比）
        let displayWidth, displayHeight;
        
        if (options.maintainAspectRatio) {
            // 根据可用空间和图像宽高比计算最佳尺寸
            const widthRatio = availableWidth / actualWidth;
            const heightRatio = availableHeight / actualHeight;
            const scale = Math.min(widthRatio, heightRatio, 1.0); // 不放大，只缩小
            
            displayWidth = actualWidth * scale;
            displayHeight = actualHeight * scale;
        } else {
            displayWidth = availableWidth;
            displayHeight = availableHeight;
        }
        
        // 确保尺寸在合理范围内
        displayWidth = Math.max(displayWidth, 100);
        displayHeight = Math.max(displayHeight, 100);
        
        // 更新图像样式
        img.style.width = displayWidth + 'px';
        img.style.height = displayHeight + 'px';
        img.style.objectFit = 'contain';
        img.style.display = 'block';
        img.style.margin = 'auto';
        img.style.maxWidth = '100%';
        img.style.maxHeight = '100%';
    }
    
    // 更新容器样式
    if (container.classList && container.classList.contains('image-placeholder')) {
        if (options.fillContainer) {
            // 占满模式：移除所有padding和居中样式
            container.style.display = 'block';
            container.style.padding = '0';
            container.style.margin = '0';
            container.style.alignItems = 'stretch';
            container.style.justifyContent = 'stretch';
        } else {
            // 适应模式：居中显示
            container.style.display = 'flex';
            container.style.alignItems = 'center';
            container.style.justifyContent = 'center';
            container.style.padding = options.padding + 'px';
            container.style.minHeight = 'auto';
        }
    }
    
    // 更新占位符文本显示图像尺寸信息
    const placeholderSubtext = container.querySelector ? container.querySelector('.placeholder-subtext') : null;
    if (placeholderSubtext && (imgWidth > 0 && imgHeight > 0)) {
        placeholderSubtext.textContent = `${imgWidth} × ${imgHeight}`;
    }
    
    // 记录优化信息（仅在开发模式下）
    if (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1') {
        if (imgWidth > 0 && imgHeight > 0) {
            if (options.fillContainer) {
                console.log(`图像显示优化: 原始尺寸 ${imgWidth}×${imgHeight}, 占满容器 ${containerWidth}×${containerHeight}, 宽高比 ${imgAspectRatio.toFixed(2)}`);
            } else {
                const finalWidth = typeof displayWidth !== 'undefined' ? displayWidth : containerWidth;
                const finalHeight = typeof displayHeight !== 'undefined' ? displayHeight : containerHeight;
                console.log(`图像显示优化: 原始尺寸 ${imgWidth}×${imgHeight}, 显示尺寸 ${Math.round(finalWidth)}×${Math.round(finalHeight)}, 宽高比 ${imgAspectRatio.toFixed(2)}`);
            }
        } else {
            console.log(`图像显示优化: 使用默认尺寸 640×480`);
        }
    }
}

// Debug选项卡状态管理
const debugState = {
    depthImage: null,              // 原始深度图（base64）
    colorImage: null,              // 原始彩色图（base64）
    binaryImage: null,             // 二值化图（base64）
    preprocessedImage: null,       // 预处理图（base64）
    refreshInterval: null,         // 刷新定时器
    isRefreshing: false,          // 是否正在刷新
    params: {                      // 处理参数
        min_depth: 0,
        max_depth: 65535,
        contour_min_area: 10,
        contour_max_area: 100000,
        min_aspect: 0.3,
        max_aspect: 4.0,
        min_width: 60,
        min_height: 60,
        max_count: 3
    }
};

// 选项卡切换功能
function switchTab(tabName) {
    // 隐藏所有选项卡内容
    const tabContents = document.querySelectorAll('.tab-content');
    tabContents.forEach(content => {
        content.classList.add('hidden');
    });

    // 移除所有按钮的active类
    const tabButtons = document.querySelectorAll('.tab-button');
    tabButtons.forEach(button => {
        button.classList.remove('active');
    });

    // 显示选中的选项卡内容
    document.getElementById(tabName + '-tab').classList.remove('hidden');
    
    // 添加active类到选中的按钮
    event.target.classList.add('active');
    
    // 如果切换到模板设置选项卡，刷新工件ID下拉框（不自动刷新模板列表）
    if (tabName === 'template') {
        refreshTemplateWorkpieceIdDropdown();
    }
    
    // 如果切换到工作流程选项卡，刷新工件ID下拉框
    if (tabName === 'workflow') {
        refreshWorkpieceIdDropdown();
    }
    
    // 如果切换到debug选项卡，加载参数并启动自动刷新
    if (tabName === 'debug') {
        loadDebugParams();
        startDebugAutoRefresh();
    } else {
        // 离开debug选项卡时停止自动刷新
        stopDebugAutoRefresh();
    }
}

// 工作流程按钮功能
// 工作流程状态
let workflowState = {
    currentImage: null,
    currentImageBase64: null,
    currentDepthImageBase64: null,  // 深度图
    currentColorImageBase64: null,  // 彩色图
    lastEstimateResult: null        // 最近一次姿态估计结果，供自动抓取使用
};

function loadImage() {
    addLogEntry('warning', '加载本地图像功能已禁用');
    alert('姿态估计现在需要深度图和彩色图，请使用"采集图像"按钮从相机获取图像。\n\n如需加载本地图像进行测试，请确保同时提供深度图（PNG格式）和彩色图（JPG格式）。');
    // TODO: 未来可以扩展为支持加载两张图像（深度图+彩色图）
    /*
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = 'image/*';
    input.onchange = function(e) {
        const file = e.target.files[0];
        if (file) {
            const reader = new FileReader();
            reader.onload = function(e) {
                const imageBase64 = e.target.result;
                workflowState.currentImageBase64 = imageBase64;
                
                const img = document.createElement('img');
                img.src = imageBase64;
                img.style.width = '100%';
                img.style.height = '100%';
                img.style.objectFit = 'contain';
                const placeholder = document.getElementById('main-image');
                placeholder.innerHTML = '';
                placeholder.appendChild(img);
                addLogEntry('success', '图像加载成功: ' + file.name);
            };
            reader.readAsDataURL(file);
        }
    };
    input.click();
    */
}

function captureImage() {
    addLogEntry('info', '正在触发相机拍照，相机ID: 207000152740');
    
    fetch(`${API_BASE_URL}/api/capture_image`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            camera_id: '207000152740'
        })
    })
    .then(async response => {
        if (!response.ok) {
            const text = await response.text();
            throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
        }
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            const text = await response.text();
            throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
        }
        return response.json();
    })
    .then(data => {
        if (data.success) {
            // 获取深度图和彩色图
            workflowState.currentDepthImageBase64 = data.depth_image_base64;
            workflowState.currentColorImageBase64 = data.color_image_base64;
            // 保持向后兼容
            workflowState.currentImageBase64 = data.color_image_base64;
            
            // 显示彩色图到图像显示区域
            const placeholder = document.getElementById('main-image');
            placeholder.innerHTML = '';
            const img = document.createElement('img');
            img.src = data.color_image_base64;
            placeholder.appendChild(img);
            // 根据图像尺寸优化显示
            optimizeImageDisplay(img, placeholder);
            
            addLogEntry('success', '图像采集完成（深度图+彩色图），图像已显示并加载等待后续操作');
        } else {
            addLogEntry('error', '图像采集失败: ' + (data.error || '未知错误'));
            alert('图像采集失败: ' + (data.error || '未知错误'));
        }
    })
    .catch(error => {
        addLogEntry('error', '图像采集异常: ' + error.message);
        console.error('图像采集错误详情:', error);
        alert('图像采集异常: ' + error.message + '\n\n请检查：\n1. 服务器是否正常运行\n2. ROS2节点是否已启动\n3. 相机服务是否可用');
    });
}

function estimatePose() {
    // 检查是否有深度图和彩色图
    if (!workflowState.currentDepthImageBase64 || !workflowState.currentColorImageBase64) {
        addLogEntry('warning', '请先采集图像（需要深度图和彩色图）');
        alert('请先采集图像（需要深度图和彩色图）');
        return;
    }
    
    const workpieceId = document.getElementById('workpiece-id-workflow').value.trim();
    if (!workpieceId) {
        addLogEntry('warning', '请选择工件ID');
        alert('请选择工件ID');
        return;
    }
    
    addLogEntry('info', `开始姿态估计，工件ID: ${workpieceId}`);
    
    fetch(`${API_BASE_URL}/api/estimate_pose`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            depth_image: workflowState.currentDepthImageBase64,
            color_image: workflowState.currentColorImageBase64,
            object_id: workpieceId
        })
    })
    .then(async response => {
        if (!response.ok) {
            const text = await response.text();
            throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
        }
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            const text = await response.text();
            throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
        }
        return response.json();
    })
    .then(data => {
        console.log('姿态估计响应数据:', data);
        
        // 检查响应格式
        if (!data) {
            addLogEntry('error', '姿态估计响应为空');
            return;
        }
        
        // 如果响应有success字段，使用它；否则假设成功（因为HTTP 200）
        const isSuccess = data.success !== false;
        
        if (isSuccess) {
            const successNum = data.success_num || 0;
            const processingTime = data.processing_time_sec !== undefined && data.processing_time_sec !== null 
                ? data.processing_time_sec.toFixed(3) 
                : null;
            
            if (processingTime !== null) {
                addLogEntry('success', `姿态估计完成，检测到 ${successNum} 个目标，处理时间: ${processingTime} 秒`);
            } else {
                addLogEntry('success', `姿态估计完成，检测到 ${successNum} 个目标`);
            }
            
            // 更新可视化图像
            if (data.vis_image) {
                const placeholder = document.getElementById('main-image');
                if (placeholder) {
                    placeholder.innerHTML = '';
                    const img = document.createElement('img');
                    img.src = data.vis_image;
                    placeholder.appendChild(img);
                    // 根据图像尺寸优化显示
                    optimizeImageDisplay(img, placeholder);
                }
            }
            
            // 更新结果列表
            updateResultCount(successNum);
            updateResultsList(data);
            workflowState.lastEstimateResult = data;

            // 输出结果信息
            for (let i = 0; i < successNum; i++) {
                const confidence = (data.confidence && data.confidence[i] !== undefined) ? data.confidence[i] : 0;
                const pos = (data.positions && data.positions[i]) ? data.positions[i] : {x: 0, y: 0, z: 0};
                const grabPos = (data.grab_positions && data.grab_positions[i]) ? data.grab_positions[i] : null;
                if (grabPos) {
                    addLogEntry('info', `目标 ${i+1}: 置信度=${(confidence * 100).toFixed(1)}%, 相机坐标=(${pos.x.toFixed(3)}, ${pos.y.toFixed(3)}, ${pos.z.toFixed(3)}), 抓取位置=(${grabPos.position.x.toFixed(3)}, ${grabPos.position.y.toFixed(3)}, ${grabPos.position.z.toFixed(3)})`);
                } else {
                    addLogEntry('info', `目标 ${i+1}: 置信度=${(confidence * 100).toFixed(1)}%, 相机坐标=(${pos.x.toFixed(3)}, ${pos.y.toFixed(3)}, ${pos.z.toFixed(3)})`);
                }
            }
        } else {
            workflowState.lastEstimateResult = null;
            addLogEntry('error', '姿态估计失败: ' + (data.error || '未知错误'));
        }
    })
    .catch(error => {
        workflowState.lastEstimateResult = null;
        addLogEntry('error', '姿态估计异常: ' + error.message);
        console.error('姿态估计错误详情:', error);
    });
}

function updateResultsList(data) {
    const resultsList = document.getElementById('results-list');
    resultsList.innerHTML = '';
    
    if (!data.success_num || data.success_num === 0) {
        resultsList.innerHTML = '<div style="padding: 20px; text-align: center; color: #64748b;">未检测到目标</div>';
        return;
    }
    
    for (let i = 0; i < data.success_num; i++) {
        const confidence = (data.confidence && data.confidence[i] !== undefined) ? data.confidence[i] : 0;
        const pos = (data.positions && data.positions[i]) ? data.positions[i] : {x: 0, y: 0, z: 0};
        const grabPos = (data.grab_positions && data.grab_positions[i]) || {};
        const prepPos = (data.preparation_positions && data.preparation_positions[i]) || {};
        const preplacePos = (data.preplace_positions && data.preplace_positions[i]) || {};
        const placePos = (data.place_positions && data.place_positions[i]) || {};
        const poseImage = (data.pose_images && data.pose_images[i]) ? data.pose_images[i] : '';
        
        // 辅助函数：格式化数值
        const fmt = (val, decimals = 2) => val !== undefined && val !== null ? val.toFixed(decimals) : '0.00';
        const fmt4 = (val) => val !== undefined && val !== null ? val.toFixed(4) : '0.0000';
        const fmtArray = (arr, decimals = 2) => {
            if (!arr || !Array.isArray(arr)) return 'N/A';
            return arr.map(v => fmt(v, decimals)).join(', ');
        };
        
        // 辅助函数：创建姿态信息块（小字体，可点击）
        const createPoseBlock = (pose, title, color, poseType) => {
            if (!pose || !pose.position) return '';
            const p = pose.position || {};
            const o = pose.orientation || {};
            const eulerDeg = pose.euler_orientation_rpy_deg || [];
            const jointsDeg = pose.joint_position_deg || [];
            
            const poseData = JSON.stringify({
                position: p,
                orientation: o,
                euler_orientation_rpy_deg: eulerDeg,
                euler_orientation_rpy_rad: pose.euler_orientation_rpy_rad || [],
                joint_position_deg: jointsDeg,
                joint_position_rad: pose.joint_position_rad || []
            });
            
            return `
                <div style="margin-bottom: 8px;">
                    <div style="color: #64748b; font-size: 0.7em; margin-bottom: 2px; font-weight: 500;">${title}</div>
                    <div class="pose-block-clickable" 
                         style="font-family: 'Courier New', monospace; color: #0f172a; background: #f8fafc; padding: 6px 8px; border-radius: 3px; border-left: 2px solid ${color}; font-size: 0.75em; line-height: 1.4; cursor: pointer; transition: all 0.2s;"
                         data-pose-type="${poseType}"
                         data-pose-data='${poseData.replace(/'/g, "&apos;")}'
                         onclick="showPoseModal('${poseType}', ${JSON.stringify(poseData).replace(/"/g, '&quot;')})">
                        <div><strong>位置:</strong> (${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.z)})</div>
                        <div><strong>姿态:</strong> (${fmt4(o.x)}, ${fmt4(o.y)}, ${fmt4(o.z)}, ${fmt4(o.w)})</div>
                        ${eulerDeg.length >= 3 ? `<div><strong>欧拉角:</strong> [${fmtArray(eulerDeg, 2)}]°</div>` : ''}
                        ${jointsDeg.length >= 6 ? `<div><strong>关节:</strong> [${fmtArray(jointsDeg, 2)}]°</div>` : ''}
                    </div>
                </div>
            `;
        };
        
        const resultItem = document.createElement('div');
        resultItem.className = 'result-item';
        resultItem.style.marginBottom = '16px';
        resultItem.style.display = 'flex';
        resultItem.style.flexDirection = 'column';
        resultItem.innerHTML = `
            <div style="width: 100%; margin-bottom: 12px;">
                ${poseImage ? `<img src="${poseImage}" style="width: 100%; max-height: 300px; object-fit: contain; border-radius: 4px; background: #f8fafc;" />` : '<div class="mini-image-placeholder" style="width: 100%; height: 200px;">📷</div>'}
            </div>
            
            <div style="width: 100%; padding: 0 4px;">
                <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; padding-bottom: 6px; border-bottom: 1px solid #e2e8f0;">
                    <div style="font-weight: 600; color: #1e293b; font-size: 0.95em;">目标 #${i + 1}</div>
                    <div style="color: #64748b; font-size: 0.8em;">
                        置信度: <span style="color: #3b82f6; font-weight: 600;">${(confidence * 100).toFixed(1)}%</span>
                        ${data.processing_time_sec !== undefined && data.processing_time_sec !== null && i === 0 ? 
                            `<span style="margin-left: 12px; color: #10b981;">处理时间: ${data.processing_time_sec.toFixed(3)}s</span>` : ''}
                    </div>
                </div>
                
                <div style="margin-bottom: 8px;">
                    <div style="color: #64748b; font-size: 0.7em; margin-bottom: 2px; font-weight: 500;">position (相机坐标)</div>
                    <div style="font-family: 'Courier New', monospace; color: #0f172a; background: #f8fafc; padding: 6px 8px; border-radius: 3px; border-left: 2px solid #3b82f6; font-size: 0.75em;">
                        (${fmt(pos.x)}, ${fmt(pos.y)}, ${fmt(pos.z)})
                    </div>
                </div>
                
                <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px;">
                    ${createPoseBlock(grabPos, 'grab_position', '#10b981', 'grab')}
                    ${createPoseBlock(prepPos, 'preparation_position', '#f59e0b', 'preparation')}
                    ${createPoseBlock(preplacePos, 'preplace_position', '#8b5cf6', 'preplace')}
                    ${createPoseBlock(placePos, 'place_position', '#ef4444', 'place')}
                </div>
            </div>
        `;
        resultsList.appendChild(resultItem);
    }
}

// 全局变量：当前选中的姿态数据
let currentPoseData = null;
let currentPoseType = '';

// 显示姿态模态框
function showPoseModal(poseType, poseDataStr) {
    // 解析poseData字符串
    let poseData;
    if (typeof poseDataStr === 'string') {
        try {
            poseData = JSON.parse(poseDataStr.replace(/&quot;/g, '"'));
        } catch (e) {
            console.error('解析姿态数据失败:', e);
            addLogEntry('error', '解析姿态数据失败');
            return;
        }
    } else {
        poseData = poseDataStr;
    }
    
    // 转换模板数据格式为工作流程格式（如果数据是模板格式）
    if (poseData.cartesian_position) {
        const cartesian = poseData.cartesian_position;
        poseData = {
            position: cartesian.position || {},
            orientation: cartesian.orientation || {},
            euler_orientation_rpy_deg: cartesian.euler_orientation_rpy_deg || [],
            euler_orientation_rpy_rad: cartesian.euler_orientation_rpy_rad || [],
            joint_position_deg: poseData.joint_position_deg || [],
            joint_position_rad: poseData.joint_position_rad || []
        };
    }
    
    currentPoseData = poseData;
    currentPoseType = poseType;
    
    const modal = document.getElementById('pose-modal');
    if (!modal) {
        // 创建模态框
        const modalHTML = `
            <div id="pose-modal" class="modal-overlay" onclick="if(event.target.id === 'pose-modal') closePoseModal()">
                <div class="modal-content" onclick="event.stopPropagation()">
                    <div class="modal-header">
                        <div class="modal-title">姿态数据 - ${getPoseTypeName(poseType)}</div>
                        <button class="modal-close" onclick="closePoseModal()">×</button>
                    </div>
                    <div class="modal-body" id="pose-modal-body">
                        <!-- 内容将动态填充 -->
                    </div>
                    <div class="modal-buttons">
                        <button class="modal-button modal-button-cartesian" onclick="executeRobotPose(false)">
                            笛卡尔坐标
                        </button>
                        <button class="modal-button modal-button-joint" onclick="executeRobotPose(true)">
                            关节坐标
                        </button>
                        <button class="modal-button modal-button-cancel" onclick="closePoseModal()">
                            取消
                        </button>
                    </div>
                </div>
            </div>
        `;
        document.body.insertAdjacentHTML('beforeend', modalHTML);
    }
    
    // 填充模态框内容
    const modalBody = document.getElementById('pose-modal-body');
    const fmt = (val, decimals = 2) => val !== undefined && val !== null ? val.toFixed(decimals) : '0.00';
    const fmt4 = (val) => val !== undefined && val !== null ? val.toFixed(4) : '0.0000';
    const fmtArray = (arr, decimals = 2) => {
        if (!arr || !Array.isArray(arr)) return 'N/A';
        return arr.map(v => fmt(v, decimals)).join(', ');
    };
    
    modalBody.innerHTML = `
        <div class="pose-data-item">
            <div class="pose-data-label">位置 (position)</div>
            <div class="pose-data-value">(${fmt(poseData.position.x)}, ${fmt(poseData.position.y)}, ${fmt(poseData.position.z)})</div>
        </div>
        <div class="pose-data-item">
            <div class="pose-data-label">姿态四元数 (orientation)</div>
            <div class="pose-data-value">(${fmt4(poseData.orientation.x)}, ${fmt4(poseData.orientation.y)}, ${fmt4(poseData.orientation.z)}, ${fmt4(poseData.orientation.w)})</div>
        </div>
        ${poseData.euler_orientation_rpy_deg && poseData.euler_orientation_rpy_deg.length >= 3 ? `
        <div class="pose-data-item">
            <div class="pose-data-label">欧拉角 (euler_orientation_rpy_deg)</div>
            <div class="pose-data-value">[${fmtArray(poseData.euler_orientation_rpy_deg, 2)}]°</div>
        </div>
        ` : ''}
        ${poseData.euler_orientation_rpy_rad && poseData.euler_orientation_rpy_rad.length >= 3 ? `
        <div class="pose-data-item">
            <div class="pose-data-label">欧拉角 (euler_orientation_rpy_rad)</div>
            <div class="pose-data-value">[${fmtArray(poseData.euler_orientation_rpy_rad, 4)}] rad</div>
        </div>
        ` : ''}
        ${poseData.joint_position_deg && poseData.joint_position_deg.length >= 6 ? `
        <div class="pose-data-item">
            <div class="pose-data-label">关节坐标 (joint_position_deg)</div>
            <div class="pose-data-value">[${fmtArray(poseData.joint_position_deg, 2)}]°</div>
        </div>
        ` : ''}
        ${poseData.joint_position_rad && poseData.joint_position_rad.length >= 6 ? `
        <div class="pose-data-item">
            <div class="pose-data-label">关节坐标 (joint_position_rad)</div>
            <div class="pose-data-value">[${fmtArray(poseData.joint_position_rad, 4)}] rad</div>
        </div>
        ` : ''}
    `;
    
    // 显示模态框
    document.getElementById('pose-modal').classList.add('show');
}

// 从模板列表显示姿态模态框（处理转义的数据）
function showPoseModalFromTemplate(poseType, poseDataStr) {
    try {
        // 还原转义的字符
        const unescaped = poseDataStr
            .replace(/&quot;/g, '"')
            .replace(/\\'/g, "'")
            .replace(/\\\\/g, '\\');
        const poseData = JSON.parse(unescaped);
        showPoseModal(poseType, poseData);
    } catch (e) {
        console.error('解析模板姿态数据失败:', e);
        addLogEntry('error', '解析模板姿态数据失败');
    }
}

// 关闭姿态模态框
function closePoseModal() {
    const modal = document.getElementById('pose-modal');
    if (modal) {
        modal.classList.remove('show');
    }
    currentPoseData = null;
    currentPoseType = '';
}

// 获取姿态类型名称
function getPoseTypeName(poseType) {
    const names = {
        'grab': '抓取姿态',
        'preparation': '准备姿态',
        'preplace': '预放置姿态',
        'place': '放置位姿'
    };
    return names[poseType] || poseType;
}

// 执行机器人位姿
async function executeRobotPose(useJoints) {
    if (!currentPoseData) {
        addLogEntry('error', '没有选中的姿态数据');
        return;
    }

    let targetPose = [];
    let isRadian = false;

    if (useJoints) {
        // 使用关节坐标
        if (currentPoseData.joint_position_deg && currentPoseData.joint_position_deg.length >= 6) {
            targetPose = currentPoseData.joint_position_deg;
            isRadian = false;
        } else if (currentPoseData.joint_position_rad && currentPoseData.joint_position_rad.length >= 6) {
            targetPose = currentPoseData.joint_position_rad;
            isRadian = true;
        } else {
            addLogEntry('error', '关节坐标数据不可用');
            return;
        }
    } else {
        // 使用笛卡尔坐标
        const pos = currentPoseData.position;
        let euler = [];
        
        if (currentPoseData.euler_orientation_rpy_deg && currentPoseData.euler_orientation_rpy_deg.length >= 3) {
            euler = currentPoseData.euler_orientation_rpy_deg;
            isRadian = false;
        } else if (currentPoseData.euler_orientation_rpy_rad && currentPoseData.euler_orientation_rpy_rad.length >= 3) {
            euler = currentPoseData.euler_orientation_rpy_rad;
            isRadian = true;
        } else {
            addLogEntry('error', '欧拉角数据不可用');
            return;
        }
        
        // target_pose格式: [x, y, z, roll, pitch, yaw]
        // z值不再限制，使用原始值
        const originalZ = pos.z !== undefined && pos.z !== null ? Number(pos.z) : 0;
        targetPose = [pos.x, pos.y, originalZ, euler[0], euler[1], euler[2]];
    }

    addLogEntry('info', `执行机器人动作: ${useJoints ? '关节坐标' : '笛卡尔坐标'}`);

    try {
        const response = await fetch('/api/set_robot_pose', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                target_pose: targetPose,
                use_joints: useJoints,
                is_radian: isRadian,
                velocity: 50.0
            })
        });

        const result = await response.json();

        if (response.ok && result.success) {
            addLogEntry('success', `机器人动作执行成功: ${result.message}`);
            closePoseModal();
        } else {
            addLogEntry('error', `机器人动作执行失败: ${result.message || result.error || '未知错误'}`);
        }
    } catch (error) {
        addLogEntry('error', `机器人动作执行异常: ${error.message}`);
        console.error('执行机器人动作错误:', error);
    }
}

function graspTarget() {
    addLogEntry('info', '开始目标抓取');
    // 模拟抓取过程
    setTimeout(() => {
        addLogEntry('success', '目标抓取完成');
    }, 1500);
}

async function openGripper() {
    addLogEntry('info', '正在打开夹爪...');
    
    try {
        const response = await fetch(`${API_BASE_URL}/api/set_robot_io`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                io_type: 'digital_output',
                io_index: 8,
                value: 0.0
            })
        });
        
        const result = await response.json();
        
        if (response.ok && result.success) {
            addLogEntry('success', '夹爪已打开');
        } else {
            addLogEntry('error', `打开夹爪失败: ${result.error || result.message || '未知错误'}`);
            alert(`打开夹爪失败: ${result.error || result.message || '未知错误'}`);
        }
    } catch (error) {
        addLogEntry('error', `打开夹爪异常: ${error.message}`);
        console.error('打开夹爪错误详情:', error);
        alert(`打开夹爪异常: ${error.message}\n\n请检查：\n1. 服务器是否正常运行\n2. ROS2节点是否已启动\n3. /demo_driver/set_io 服务是否可用`);
    }
}

async function closeGripper() {
    addLogEntry('info', '正在关闭夹爪...');
    
    try {
        const response = await fetch(`${API_BASE_URL}/api/set_robot_io`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                io_type: 'digital_output',
                io_index: 8,
                value: 1.0
            })
        });
        
        const result = await response.json();
        
        if (response.ok && result.success) {
            addLogEntry('success', '夹爪已关闭');
        } else {
            addLogEntry('error', `关闭夹爪失败: ${result.error || result.message || '未知错误'}`);
            alert(`关闭夹爪失败: ${result.error || result.message || '未知错误'}`);
        }
    } catch (error) {
        addLogEntry('error', `关闭夹爪异常: ${error.message}`);
        console.error('关闭夹爪错误详情:', error);
        alert(`关闭夹爪异常: ${error.message}\n\n请检查：\n1. 服务器是否正常运行\n2. ROS2节点是否已启动\n3. /demo_driver/set_io 服务是否可用`);
    }
}

/**
 * 执行姿态序列
 * @param {string} folderName - 文件夹名称，例如 "0_toCam"
 */
async function executePoseSequence(folderName) {
    if (!folderName) {
        addLogEntry('error', '文件夹名称不能为空');
        return;
    }
    
    addLogEntry('info', `开始执行姿态序列: ${folderName}`);
    
    try {
        const response = await fetch(`${API_BASE_URL}/api/execute_pose_sequence`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                folder_name: folderName
            })
        });
        
        if (!response.ok) {
            const text = await response.text();
            throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
        }
        
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            const text = await response.text();
            throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
        }
        
        const data = await response.json();
        
        if (data.success) {
            if (data.skipped) {
                addLogEntry('warning', data.message || '已跳过执行');
                addLogEntry('info', `当前关节角度: [${data.current_joints.map(v => v.toFixed(3)).join(', ')}]`);
                addLogEntry('info', `最后点位角度: [${data.last_pose.map(v => v.toFixed(3)).join(', ')}]`);
            } else {
                addLogEntry('success', data.message || '姿态序列执行已开始');
                addLogEntry('info', `共 ${data.total_files} 个点位文件`);
                addLogEntry('info', `当前关节角度: [${data.current_joints.map(v => v.toFixed(3)).join(', ')}]`);
            }
        } else {
            addLogEntry('error', `执行失败: ${data.error || '未知错误'}`);
        }
    } catch (error) {
        addLogEntry('error', `执行姿态序列异常: ${error.message}`);
        console.error('执行姿态序列错误详情:', error);
    }
}

const _delayMs = (ms) => new Promise(r => setTimeout(r, ms));

/** 夹爪打开函数 */
async function _gripperOpenPlaceholder() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/set_robot_io`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                io_type: 'digital_output',
                io_index: 8,
                value: 0.0
            })
        });
        
        const result = await response.json();
        
        if (response.ok && result.success) {
            addLogEntry('success', '夹爪已打开');
            return true;
        } else {
            addLogEntry('error', `打开夹爪失败: ${result.error || result.message || '未知错误'}`);
            return false;
        }
    } catch (error) {
        addLogEntry('error', `打开夹爪异常: ${error.message}`);
        return false;
    }
}

/** 夹爪关闭函数 */
async function _gripperClosePlaceholder() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/set_robot_io`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                io_type: 'digital_output',
                io_index: 8,
                value: 1.0
            })
        });
        
        const result = await response.json();
        
        if (response.ok && result.success) {
            addLogEntry('success', '夹爪已关闭');
            return true;
        } else {
            addLogEntry('error', `关闭夹爪失败: ${result.error || result.message || '未知错误'}`);
            return false;
        }
    } catch (error) {
        addLogEntry('error', `关闭夹爪异常: ${error.message}`);
        return false;
    }
}

let _autoGraspRunning = false;
let _loopAutoGraspRunning = false;
let _loopAutoGraspStopFlag = false;

/**
 * Promise包装的采集图像函数
 * @returns {Promise<{success: boolean, error?: string}>}
 */
async function captureImageAsync() {
    return new Promise((resolve, reject) => {
        addLogEntry('info', '正在触发相机拍照，相机ID: 207000152740');
        
        fetch(`${API_BASE_URL}/api/capture_image`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                camera_id: '207000152740'
            })
        })
        .then(async response => {
            if (!response.ok) {
                const text = await response.text();
                throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
            }
            const contentType = response.headers.get('content-type');
            if (!contentType || !contentType.includes('application/json')) {
                const text = await response.text();
                throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
            }
            return response.json();
        })
        .then(data => {
            if (data.success) {
                // 获取深度图和彩色图
                workflowState.currentDepthImageBase64 = data.depth_image_base64;
                workflowState.currentColorImageBase64 = data.color_image_base64;
                // 保持向后兼容
                workflowState.currentImageBase64 = data.color_image_base64;
                
                // 显示彩色图到图像显示区域
                const placeholder = document.getElementById('main-image');
                placeholder.innerHTML = '';
                const img = document.createElement('img');
                img.src = data.color_image_base64;
                placeholder.appendChild(img);
                // 根据图像尺寸优化显示
                optimizeImageDisplay(img, placeholder);
                
                addLogEntry('success', '图像采集完成（深度图+彩色图）');
                resolve({ success: true });
            } else {
                const errorMsg = data.error || '未知错误';
                addLogEntry('error', '图像采集失败: ' + errorMsg);
                resolve({ success: false, error: errorMsg });
            }
        })
        .catch(error => {
            addLogEntry('error', '图像采集异常: ' + error.message);
            console.error('图像采集错误详情:', error);
            resolve({ success: false, error: error.message });
        });
    });
}

/**
 * Promise包装的姿态估计函数
 * @returns {Promise<{success: boolean, success_num?: number, error?: string}>}
 */
async function estimatePoseAsync() {
    return new Promise((resolve, reject) => {
        // 检查是否有深度图和彩色图
        if (!workflowState.currentDepthImageBase64 || !workflowState.currentColorImageBase64) {
            addLogEntry('warning', '请先采集图像（需要深度图和彩色图）');
            resolve({ success: false, error: '缺少图像数据' });
            return;
        }
        
        const workpieceId = document.getElementById('workpiece-id-workflow').value.trim();
        if (!workpieceId) {
            addLogEntry('warning', '请选择工件ID');
            resolve({ success: false, error: '缺少工件ID' });
            return;
        }
        
        addLogEntry('info', `开始姿态估计，工件ID: ${workpieceId}`);
        
        fetch(`${API_BASE_URL}/api/estimate_pose`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                depth_image: workflowState.currentDepthImageBase64,
                color_image: workflowState.currentColorImageBase64,
                object_id: workpieceId
            })
        })
        .then(async response => {
            if (!response.ok) {
                const text = await response.text();
                throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
            }
            const contentType = response.headers.get('content-type');
            if (!contentType || !contentType.includes('application/json')) {
                const text = await response.text();
                throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
            }
            return response.json();
        })
        .then(data => {
            console.log('姿态估计响应数据:', data);
            
            // 检查响应格式
            if (!data) {
                addLogEntry('error', '姿态估计响应为空');
                resolve({ success: false, error: '响应为空' });
                return;
            }
            
            // 如果响应有success字段，使用它；否则假设成功（因为HTTP 200）
            const isSuccess = data.success !== false;
            
            if (isSuccess) {
                const successNum = data.success_num || 0;
                const processingTime = data.processing_time_sec !== undefined && data.processing_time_sec !== null 
                    ? data.processing_time_sec.toFixed(3) 
                    : null;
                
                if (processingTime !== null) {
                    addLogEntry('success', `姿态估计完成，检测到 ${successNum} 个目标，处理时间: ${processingTime} 秒`);
                } else {
                    addLogEntry('success', `姿态估计完成，检测到 ${successNum} 个目标`);
                }
                
                // 更新可视化图像
                if (data.vis_image) {
                    const placeholder = document.getElementById('main-image');
                    if (placeholder) {
                        placeholder.innerHTML = '';
                        const img = document.createElement('img');
                        img.src = data.vis_image;
                        placeholder.appendChild(img);
                        // 根据图像尺寸优化显示
                        optimizeImageDisplay(img, placeholder);
                    }
                }
                
                // 更新结果列表
                updateResultCount(successNum);
                updateResultsList(data);
                workflowState.lastEstimateResult = data;

                // 输出结果信息
                for (let i = 0; i < successNum; i++) {
                    const confidence = (data.confidence && data.confidence[i] !== undefined) ? data.confidence[i] : 0;
                    const pos = (data.positions && data.positions[i]) ? data.positions[i] : {x: 0, y: 0, z: 0};
                    const grabPos = (data.grab_positions && data.grab_positions[i]) ? data.grab_positions[i] : null;
                    if (grabPos) {
                        addLogEntry('info', `目标 ${i+1}: 置信度=${(confidence * 100).toFixed(1)}%, 相机坐标=(${pos.x.toFixed(3)}, ${pos.y.toFixed(3)}, ${pos.z.toFixed(3)}), 抓取位置=(${grabPos.position.x.toFixed(3)}, ${grabPos.position.y.toFixed(3)}, ${grabPos.position.z.toFixed(3)})`);
                    } else {
                        addLogEntry('info', `目标 ${i+1}: 置信度=${(confidence * 100).toFixed(1)}%, 相机坐标=(${pos.x.toFixed(3)}, ${pos.y.toFixed(3)}, ${pos.z.toFixed(3)})`);
                    }
                }
                
                resolve({ success: true, success_num: successNum });
            } else {
                workflowState.lastEstimateResult = null;
                const errorMsg = data.error || '未知错误';
                addLogEntry('error', '姿态估计失败: ' + errorMsg);
                resolve({ success: false, error: errorMsg });
            }
        })
        .catch(error => {
            workflowState.lastEstimateResult = null;
            addLogEntry('error', '姿态估计异常: ' + error.message);
            console.error('姿态估计错误详情:', error);
            resolve({ success: false, error: error.message });
        });
    });
}

/**
 * 循环自动抓取函数
 * 在拍照姿态下每隔1s刷新一次图像，位姿估计识别到工件后进行自动抓取，
 * 等回到拍照姿态后继续刷新图像进行位姿检测，循环检测自动抓取
 */
async function loopAutoGrasp() {
    if (_loopAutoGraspRunning) {
        addLogEntry('warning', '循环自动抓取已在运行中');
        return;
    }

    const workpieceId = document.getElementById('workpiece-id-workflow').value.trim();
    if (!workpieceId) {
        addLogEntry('error', '循环自动抓取失败：请先选择工件ID');
        alert('请先选择工件ID');
        return;
    }

    _loopAutoGraspRunning = true;
    _loopAutoGraspStopFlag = false;
    
    // 更新按钮状态
    const btn = document.getElementById('loop-auto-grasp-btn');
    if (btn) {
        btn.textContent = '⏹️\n停止循环';
        btn.className = 'btn btn-danger';
    }

    addLogEntry('info', '开始循环自动抓取模式（请确保机械臂已在拍照姿态）');
    
    try {
        // 循环检测和抓取
        let cycleCount = 0;
        while (!_loopAutoGraspStopFlag) {
            cycleCount++;
            addLogEntry('info', `========== 循环第 ${cycleCount} 次 ==========`);

            // 检查停止标志
            if (_loopAutoGraspStopFlag) {
                addLogEntry('info', '收到停止信号，退出循环');
                break;
            }

            // 步骤1: 采集图像
            addLogEntry('info', '采集图像...');
            const captureResult = await captureImageAsync();
            if (!captureResult.success) {
                addLogEntry('warning', '图像采集失败，等待1秒后重试');
                await _delayMs(1000);
                continue;
            }

            // 检查停止标志
            if (_loopAutoGraspStopFlag) {
                addLogEntry('info', '收到停止信号，退出循环');
                break;
            }

            // 等待1秒后刷新图像（符合"每隔1s刷新一次图像"的要求）
            await _delayMs(1000);

            // 检查停止标志
            if (_loopAutoGraspStopFlag) {
                addLogEntry('info', '收到停止信号，退出循环');
                break;
            }

            // 步骤2: 姿态估计
            addLogEntry('info', '进行姿态估计...');
            const estimateResult = await estimatePoseAsync();
            if (!estimateResult.success) {
                addLogEntry('warning', '姿态估计失败，继续下一轮循环');
                continue;
            }

            // 检查停止标志
            if (_loopAutoGraspStopFlag) {
                addLogEntry('info', '收到停止信号，退出循环');
                break;
            }

            // 步骤3: 检查是否检测到工件
            const successNum = estimateResult.success_num || 0;
            if (successNum > 0) {
                addLogEntry('info', `检测到 ${successNum} 个工件，开始自动抓取...`);
                
                // 执行自动抓取（autoGrasp内部会回到拍照姿态）
                const graspSuccess = await autoGrasp();
                
                // 如果自动抓取失败（包括机械臂移动失败），退出循环
                if (!graspSuccess) {
                    addLogEntry('error', '自动抓取失败（机械臂移动失败），退出循环自动抓取');
                    break;
                }
                
                // 自动抓取完成后，等待回到拍照姿态稳定
                addLogEntry('info', '等待回到拍照姿态稳定...');
                await _delayMs(500);
            } else {
                addLogEntry('info', '未检测到工件，继续下一轮循环');
            }
        }

        addLogEntry('success', `循环自动抓取已停止，共执行 ${cycleCount} 次循环`);
    } catch (e) {
        addLogEntry('error', '循环自动抓取异常: ' + e.message);
        console.error('循环自动抓取错误:', e);
    } finally {
        _loopAutoGraspRunning = false;
        _loopAutoGraspStopFlag = false;
        
        // 恢复按钮状态
        const btn = document.getElementById('loop-auto-grasp-btn');
        if (btn) {
            btn.textContent = '🔄\n循环抓取';
            btn.className = 'btn btn-success';
        }
    }
}

/**
 * 停止循环自动抓取
 */
function stopLoopAutoGrasp() {
    if (!_loopAutoGraspRunning) {
        addLogEntry('warning', '循环自动抓取未在运行');
        return;
    }
    
    addLogEntry('info', '正在停止循环自动抓取...');
    _loopAutoGraspStopFlag = true;
}

/**
 * 循环自动抓取按钮点击处理函数
 */
function toggleLoopAutoGrasp() {
    if (_loopAutoGraspRunning) {
        stopLoopAutoGrasp();
    } else {
        loopAutoGrasp();
    }
}

/**
 * 自动抓取函数
 * @returns {Promise<boolean>} 返回true表示成功，false表示失败（包括移动失败）
 */
async function autoGrasp() {
    const d = workflowState.lastEstimateResult;
    const n = d && (d.success_num || 0);

    if (!n) {
        alert('请先执行姿态估计');
        addLogEntry('warning', '自动抓取失败：请先执行姿态估计');
        return false;
    }

    const prep = (d.preparation_positions && d.preparation_positions[0]) || null;
    const grab = (d.grab_positions && d.grab_positions[0]) || null;
    const preplace = (d.preplace_positions && d.preplace_positions[0]) || null;
    const place = (d.place_positions && d.place_positions[0]) || null;

    if (!prep?.position || !grab?.position) {
        addLogEntry('error', '自动抓取失败：缺少准备姿态或抓取姿态');
        return false;
    }

    if (!preplace?.position || !place?.position) {
        addLogEntry('warning', '缺少预放置姿态或放置姿态，将跳过放置步骤');
    }

    if (_autoGraspRunning) {
        addLogEntry('warning', '自动抓取进行中，请勿重复点击');
        return false;
    }
    _autoGraspRunning = true;

    addLogEntry('info', `开始自动抓取，目标 1/${n}（回到拍照→打开夹爪→准备→抓取→关闭夹爪→准备→预放置→放置→打开夹爪→回到拍照）`);

    const v = 0.1;
    const a = 0.025;

    // 辅助函数：回到拍照姿态
    async function moveToCameraPose() {
        try {
            const workpieceId = (document.getElementById('workpiece-id-workflow')?.value || '').trim();
            const matchedPoseId = (d.matched_pose_ids && d.matched_pose_ids[0]) ? String(d.matched_pose_ids[0]) : '';

            if (!workpieceId) {
                addLogEntry('warning', '跳过回到拍照姿态：缺少工件ID');
                return false;
            } else if (!matchedPoseId) {
                addLogEntry('warning', '跳过回到拍照姿态：响应缺少 matched_pose_ids[0]');
                return false;
            } else {
                addLogEntry('info', `回到拍照姿态：使用匹配模板 pose_id=${matchedPoseId} 的 camera_pose`);
                const response = await fetch(`${API_BASE_URL}/api/read_template_pose`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        workpiece_id: workpieceId,
                        pose_id: matchedPoseId,
                        pose_type: 'camera_pose'
                    })
                });

                const data = await response.json().catch(() => ({}));
                const poseData = data && data.pose_data;
                const cart = poseData && poseData.cartesian_position;
                const pos = cart && cart.position;
                const ori = cart && cart.orientation;

                if (!response.ok || !data.success || !pos || !ori) {
                    const errorMsg = data && data.error ? data.error : '未知错误';
                    addLogEntry('warning', `跳过回到拍照姿态：无法读取拍照姿态（camera_pose），错误: ${errorMsg}`);
                    return false;
                } else {
                    const targetPose = {
                        position: { x: pos.x, y: pos.y, z: pos.z },
                        orientation: { x: ori.x, y: ori.y, z: ori.z, w: ori.w }
                    };
                    addLogEntry('info', '移动到拍照姿态...');
                    const { ok: okCam, error: errCam } = await moveRobotToPoseCartesian(targetPose, v, a, '拍照姿态');
                    if (!okCam) {
                        addLogEntry('warning', '回到拍照姿态失败: ' + (errCam || '未知错误'));
                        return false;
                    } else {
                        addLogEntry('success', '已回到拍照姿态');
                        return true;
                    }
                }
            }
        } catch (e) {
            addLogEntry('warning', '回到拍照姿态异常: ' + e.message);
            return false;
        }
    }

    try {
        // // 步骤1: 回到拍照姿态
        // await moveToCameraPose();
        // await _delayMs(500);

        // 步骤2: 打开夹爪
        const okOpen = await _gripperOpenPlaceholder();
        if (!okOpen) {
            addLogEntry('error', '打开夹爪失败');
            return false;
        }
        /* 占位函数内已延时 500ms，此处不再重复 */

        // 步骤3: 移动到准备姿态
        addLogEntry('info', '移动到准备姿态...');
        const { ok: okPrep, error: errPrep } = await moveRobotToPoseCartesian(prep, v, a, '准备姿态');
        if (!okPrep) {
            addLogEntry('error', '移动到准备姿态失败: ' + (errPrep || '未知错误'));
            return false;
        }
        addLogEntry('success', '已到达准备姿态');
        await _delayMs(500);

        // 步骤4: 移动到抓取姿态
        addLogEntry('info', '移动到抓取姿态...');
        const { ok: okGrab, error: errGrab } = await moveRobotToPoseCartesian(grab, v, a, '抓取姿态');
        if (!okGrab) {
            addLogEntry('error', '移动到抓取姿态失败: ' + (errGrab || '未知错误'));
            return false;
        }
        addLogEntry('success', '已到达抓取姿态');
        await _delayMs(300);

        // 步骤5: 关闭夹爪
        const okClose = await _gripperClosePlaceholder();
        if (!okClose) {
            addLogEntry('error', '关闭夹爪失败');
            return false;
        }
        await _delayMs(300);

        // 步骤6: 移动到准备姿态（抓取后先回到准备姿态）
        addLogEntry('info', '移动到准备姿态...');
        const { ok: okPrep2, error: errPrep2 } = await moveRobotToPoseCartesian(prep, v, a, '准备姿态');
        if (!okPrep2) {
            addLogEntry('error', '移动到准备姿态失败: ' + (errPrep2 || '未知错误'));
            return false;
        }
        addLogEntry('success', '已到达准备姿态');
        await _delayMs(500);

        // 步骤7: 移动到预放置姿态（如果存在）
        if (preplace?.position) {
            addLogEntry('info', '移动到预放置姿态...');
            const { ok: okPreplace, error: errPreplace } = await moveRobotToPoseCartesian(preplace, v, a, '预放置姿态');
            if (!okPreplace) {
                addLogEntry('error', '移动到预放置姿态失败: ' + (errPreplace || '未知错误'));
                return false;
            }
            addLogEntry('success', '已到达预放置姿态');
            await _delayMs(500);
        } else {
            addLogEntry('warning', '跳过预放置姿态（未提供）');
        }

        // 步骤8: 移动到放置姿态（如果存在）
        if (place?.position) {
            addLogEntry('info', '移动到放置姿态...');
            const { ok: okPlace, error: errPlace } = await moveRobotToPoseCartesian(place, v, a, '放置姿态');
            if (!okPlace) {
                addLogEntry('error', '移动到放置姿态失败: ' + (errPlace || '未知错误'));
                return false;
            }
            addLogEntry('success', '已到达放置姿态');
            await _delayMs(300);
        } else {
            addLogEntry('warning', '跳过放置姿态（未提供）');
        }

        // 步骤9: 打开夹爪
        const okOpen1 = await _gripperOpenPlaceholder();
        if (!okOpen1) {
            addLogEntry('error', '打开夹爪失败');
            return false;
        }
        await _delayMs(300);

        // 步骤10: 回到拍照姿态
        const okCamera = await moveToCameraPose();
        if (!okCamera) {
            addLogEntry('error', '回到拍照姿态失败');
            return false;
        }

        addLogEntry('success', '自动抓取完成');
        return true;
    } catch (e) {
        addLogEntry('error', '自动抓取异常: ' + e.message);
        console.error('自动抓取错误:', e);
        return false;
    } finally {
        _autoGraspRunning = false;
    }
}

/**
 * 将四元数转换为欧拉角（RPY，单位：度）
 * @param {Object} q - 四元数 {x, y, z, w}
 * @returns {Array} - [roll, pitch, yaw] 单位：度
 */
function quaternionToEulerDeg(q) {
    // Roll (x-axis rotation)
    const sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    const sinp = 2 * (q.w * q.y - q.z * q.x);
    let pitch;
    if (Math.abs(sinp) >= 1)
        pitch = Math.sign(sinp) * Math.PI / 2; // use 90 degrees if out of range
    else
        pitch = Math.asin(sinp);

    // Yaw (z-axis rotation)
    const siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);

    // Convert from radians to degrees
    return [
        roll * 180 / Math.PI,
        pitch * 180 / Math.PI,
        yaw * 180 / Math.PI
    ];
}

/**
 * 按笛卡尔位姿移动机械臂（与拍照姿态相同的实现方式）
 * 使用 configs/camera_pose.json 同款的 position + orientation 格式，调用 /api/set_robot_pose
 * @param {Object} pose - { position: {x,y,z}, orientation: {x,y,z,w} }
 * @param {number} velocityFactor - 速度因子，默认 0.1
 * @param {number} accelerationFactor - 加速度因子，默认 0.025
 * @param {string} [label] - 可选，位姿标签（如「拍照姿态」「准备姿态」「抓取姿态」），用于日志
 * @returns {Promise<{ok: boolean, result?: object, error?: string}>}
 */
async function moveRobotToPoseCartesian(pose, velocityFactor = 0.1, accelerationFactor = 0.025, label = '') {
    const pos = pose.position || {};
    const ori = pose.orientation || {};
    
    // z值不再限制，使用原始值
    const originalZ = pos.z !== undefined && pos.z !== null ? Number(pos.z) : 0;
    
    const targetPose = {
        position: { x: pos.x, y: pos.y, z: originalZ },
        orientation: { x: ori.x, y: ori.y, z: ori.z, w: ori.w }
    };
    const fmt = (v) => (v !== undefined && v !== null ? Number(v).toFixed(3) : '?');
    const posStr = `位置(${fmt(pos.x)}, ${fmt(pos.y)}, ${fmt(originalZ)})m`;
    const oriStr = `四元数(${fmt(ori.x)}, ${fmt(ori.y)}, ${fmt(ori.z)}, ${fmt(ori.w)})`;
    
    // 计算欧拉角
    const euler = quaternionToEulerDeg(ori);
    const eulerStr = `, 姿态角(${euler[0].toFixed(2)}, ${euler[1].toFixed(2)}, ${euler[2].toFixed(2)})deg`;
    
    addLogEntry('info', label ? `即将运动到【${label}】: ${posStr}, ${oriStr}${eulerStr}` : `即将运动到位姿: ${posStr}, ${oriStr}${eulerStr}`);

    const res = await fetch(`${API_BASE_URL}/api/set_robot_pose`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            target_pose: targetPose,
            use_joints: false,
            velocity_factor: velocityFactor,
            acceleration_factor: accelerationFactor
        })
    });
    const result = await res.json();
    return { ok: res.ok && result.success, result, error: result.error || result.message };
}

// 通用函数：从模板读取位姿并运动到该位姿
// @param {string} poseType - 位姿类型: 'camera_pose', 'preparation_position', 'grab_position', 'preplace_position', 'place_position'
// @param {string} poseLabel - 位姿标签，用于日志显示
// @param {boolean} fromTemplateTab - 是否从模板设置选项卡获取ID（true）还是从工作流程选项卡（false）
async function moveToTemplatePose(poseType, poseLabel, fromTemplateTab = true) {
    // 根据来源获取工件ID和姿态ID
    let workpieceId, poseId;
    if (fromTemplateTab) {
        workpieceId = document.getElementById('workpiece-id').value.trim();
        poseId = document.getElementById('pose-id').value.trim();
    } else {
        workpieceId = document.getElementById('workpiece-id-workflow').value.trim();
        poseId = document.getElementById('pose-id-workflow').value.trim();
    }
    
    if (!workpieceId || !poseId) {
        addLogEntry('warning', '请先选择工件ID和姿态ID');
        alert('请先选择工件ID和姿态ID');
        return;
    }
    
    addLogEntry('info', `正在读取模板${poseLabel}: 工件ID=${workpieceId}, 姿态ID=${poseId}...`);

    try {
        // 从模板读取位姿
        const response = await fetch(`${API_BASE_URL}/api/read_template_pose`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                workpiece_id: workpieceId,
                pose_id: poseId,
                pose_type: poseType
            })
        });
        
        if (!response.ok) {
            const text = await response.text();
            throw new Error(`无法读取模板${poseLabel}: HTTP ${response.status}: ${text.substring(0, 100)}`);
        }
        
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            const text = await response.text();
            throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
        }
        
        const data = await response.json();
        
        if (!data.success || !data.pose_data) {
            throw new Error(data.error || `无法获取${poseLabel}数据`);
        }
        
        const poseData = data.pose_data;

        if (!poseData.cartesian_position ||
            !poseData.cartesian_position.position ||
            !poseData.cartesian_position.orientation) {
            addLogEntry('error', `模板${poseLabel}数据格式错误：缺少有效的笛卡尔坐标和四元数数据`);
            return;
        }

        const pos = poseData.cartesian_position.position;
        const ori = poseData.cartesian_position.orientation;
        const euler = poseData.cartesian_position.euler_orientation_rpy_deg || [];
        const targetPose = {
            position: { x: pos.x, y: pos.y, z: pos.z },
            orientation: { x: ori.x, y: ori.y, z: ori.z, w: ori.w }
        };

        const { ok, error } = await moveRobotToPoseCartesian(targetPose, 0.1, 0.025, poseLabel);
        if (ok) {
            const fmt = (v) => (v !== undefined && v !== null ? Number(v).toFixed(3) : '?');
            const posStr = `位置(${fmt(pos.x)}, ${fmt(pos.y)}, ${fmt(pos.z)})m`;
            const oriStr = `四元数(${fmt(ori.x)}, ${fmt(ori.y)}, ${fmt(ori.z)}, ${fmt(ori.w)})`;
            const eulerStr = euler.length >= 3 ? `, 姿态角(${euler[0].toFixed(2)}, ${euler[1].toFixed(2)}, ${euler[2].toFixed(2)})deg` : '';
            addLogEntry('success', `机器人已运动到${poseLabel} (工件ID: ${workpieceId}, 姿态ID: ${poseId}) - ${posStr}, ${oriStr}${eulerStr}`);
        } else {
            addLogEntry('error', '机器人运动失败: ' + (error || '未知错误'));
        }
    } catch (err) {
        addLogEntry('error', `${poseLabel}运动异常: ` + err.message);
        console.error(`${poseLabel}运动错误详情:`, err);
        alert(`${poseLabel}运动异常: ${err.message}\n\n请检查：\n1. 模板是否存在\n2. ${poseLabel}数据是否完整\n3. 服务器是否正常运行`);
    }
}

// 从工作流程选项卡运动到拍照姿态
async function moveToCameraPose() {
    await moveToTemplatePose('camera_pose', '拍照姿态', false);
}

// 从模板设置选项卡运动到拍照姿态
async function moveToCameraPoseFromTemplate() {
    await moveToTemplatePose('camera_pose', '拍照姿态', true);
}

// 从模板设置选项卡运动到准备位姿
async function moveToPreparationPose() {
    await moveToTemplatePose('preparation_position', '准备位姿', true);
}

// 从模板设置选项卡运动到抓取位姿
async function moveToGrabPose() {
    await moveToTemplatePose('grab_position', '抓取位姿', true);
}

// 从模板设置选项卡运动到预放置位姿
async function moveToPreplacePose() {
    await moveToTemplatePose('preplace_position', '预放置位姿', true);
}

// 从模板设置选项卡运动到放置位姿
async function moveToPlacePose() {
    await moveToTemplatePose('place_position', '放置位姿', true);
}

// 更新结果计数
function updateResultCount(count) {
    document.getElementById('result-count').textContent = count;
}

// 模板管理功能
function refreshTemplateList() {
    const workpieceId = document.getElementById('workpiece-id').value.trim();
    
    if (!workpieceId) {
        addLogEntry('warning', '请先选择或输入工件ID');
        alert('请先选择或输入工件ID');
        return;
    }
    
    addLogEntry('info', `刷新模板列表，工件ID: ${workpieceId}`);
    
    // 调用后端API列出模板，传入工件ID过滤
    fetch(`${API_BASE_URL}/api/list_templates`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            templates_dir: '',
            workpiece_id: workpieceId  // 传入工件ID进行过滤
        })
    })
    .then(async response => {
        if (!response.ok) {
            const text = await response.text();
            throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
        }
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            const text = await response.text();
            throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
        }
        return response.json();
    })
    .then(data => {
        if (data.success) {
            // 过滤出匹配的模板
            let filteredTemplates = data.templates.filter(t => t.workpiece_id === workpieceId);
            
            // 按姿态ID增序排序
            filteredTemplates.sort((a, b) => {
                const poseIdA = a.pose_id || '';
                const poseIdB = b.pose_id || '';
                
                // 尝试将姿态ID转换为数字进行排序
                const numA = parseFloat(poseIdA);
                const numB = parseFloat(poseIdB);
                
                // 如果两个都是有效数字，按数字排序
                if (!isNaN(numA) && !isNaN(numB)) {
                    return numA - numB;
                }
                
                // 否则按字符串排序
                return poseIdA.localeCompare(poseIdB, undefined, { numeric: true, sensitivity: 'base' });
            });
            
            const count = filteredTemplates.length;
            
            addLogEntry('success', `找到 ${count} 个模板`);
            
            // 更新模板列表显示
            const templateListContainer = document.getElementById('template-results-list');
            templateListContainer.innerHTML = '';  // 清空现有列表
            
            // 更新模板数量
            document.getElementById('template-count').textContent = count;
            
            // 显示每个模板（异步加载姿态数据）
            filteredTemplates.forEach(async (template, index) => {
                // 先创建模板项的基本结构
                const templateItem = document.createElement('div');
                templateItem.className = 'result-item';
                templateItem.style.marginBottom = '16px';
                templateItem.style.display = 'flex';
                templateItem.style.flexDirection = 'column';
                templateItem.setAttribute('data-workpiece-id', template.workpiece_id);
                templateItem.setAttribute('data-pose-id', template.pose_id);
                
                // 显示加载中状态
                templateItem.innerHTML = `
                    <div style="width: 100%; margin-bottom: 12px;">
                        <img src="${template.image_base64}" style="width: 100%; max-height: 300px; object-fit: contain; border-radius: 4px; background: #f8fafc;" />
                    </div>
                    <div style="width: 100%; padding: 0 4px;">
                        <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; padding-bottom: 6px; border-bottom: 1px solid #e2e8f0;">
                            <div style="font-weight: 600; color: #1e293b; font-size: 0.95em;">模板: ${template.workpiece_id} / ${template.pose_id}</div>
                            <div style="color: #64748b; font-size: 0.8em;">加载中...</div>
                        </div>
                    </div>
                `;
                
                templateListContainer.appendChild(templateItem);
                
                // 异步加载所有姿态数据
                try {
                    const poseTypes = [
                        { type: 'preparation_position', name: '准备姿态', color: '#f59e0b', key: 'preparation' },
                        { type: 'grab_position', name: '抓取姿态', color: '#10b981', key: 'grab' },
                        { type: 'preplace_position', name: '预放置姿态', color: '#8b5cf6', key: 'preplace' },
                        { type: 'place_position', name: '放置位姿', color: '#ef4444', key: 'place' }
                    ];
                    
                    const poses = {};
                    const loadPromises = poseTypes.map(async ({ type, key }) => {
                        try {
                            const response = await fetch(`${API_BASE_URL}/api/read_template_pose`, {
                                method: 'POST',
                                headers: { 'Content-Type': 'application/json' },
                                body: JSON.stringify({
                                    workpiece_id: template.workpiece_id,
                                    pose_id: template.pose_id,
                                    pose_type: type
                                })
                            });
                            
                            if (response.ok) {
                                const data = await response.json();
                                if (data.success && data.pose_data) {
                                    poses[type] = data.pose_data;
                                    poses[type]._poseKey = key; // 保存姿态类型key
                                }
                            }
                        } catch (error) {
                            console.error(`加载姿态 ${type} 失败:`, error);
                        }
                    });
                    
                    await Promise.all(loadPromises);
                    
                    // 辅助函数：格式化数值
                    const fmt = (val, decimals = 2) => val !== undefined && val !== null ? val.toFixed(decimals) : 'N/A';
                    const fmt4 = (val) => val !== undefined && val !== null ? val.toFixed(4) : 'N/A';
                    const fmtArray = (arr, decimals = 2) => {
                        if (!arr || !Array.isArray(arr)) return 'N/A';
                        return arr.map(v => fmt(v, decimals)).join(', ');
                    };
                    
                    // 辅助函数：创建姿态信息块
                    const createPoseBlock = (poseData, title, color, poseTypeKey) => {
                        if (!poseData || !poseData.cartesian_position) return '';
                        
                        const cartesian = poseData.cartesian_position || {};
                        const p = cartesian.position || {};
                        const eulerDeg = cartesian.euler_orientation_rpy_deg || [];
                        const jointsDeg = poseData.joint_position_deg || [];
                        
                        // 准备传递给模态窗口的数据（转义特殊字符）
                        const poseDataEscaped = JSON.stringify(poseData)
                            .replace(/\\/g, '\\\\')
                            .replace(/'/g, "\\'")
                            .replace(/"/g, '&quot;');
                        
                        return `
                            <div style="margin-bottom: 10px; padding: 8px; background: #f8fafc; border-radius: 4px; border-left: 3px solid ${color}; cursor: pointer; transition: all 0.2s;"
                                 onmouseover="this.style.background='#f1f5f9'; this.style.transform='translateX(2px)'"
                                 onmouseout="this.style.background='#f8fafc'; this.style.transform='translateX(0)'"
                                 onclick="showPoseModalFromTemplate('${poseTypeKey}', '${poseDataEscaped}')">
                                <div style="font-weight: 600; color: ${color}; font-size: 0.85em; margin-bottom: 6px;">${title}</div>
                                <div style="font-family: 'Courier New', monospace; font-size: 0.75em; line-height: 1.6; color: #475569;">
                                    <div><strong>笛卡尔坐标:</strong></div>
                                    <div style="margin-left: 12px; margin-bottom: 4px;">
                                        位置: (${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.z)})
                                    </div>
                                    ${eulerDeg.length >= 3 ? `<div style="margin-left: 12px;">欧拉角: [${fmtArray(eulerDeg, 2)}]°</div>` : ''}
                                    ${jointsDeg.length >= 6 ? `<div style="margin-top: 6px;"><strong>关节坐标:</strong></div><div style="margin-left: 12px;">[${fmtArray(jointsDeg, 2)}]°</div>` : ''}
                                </div>
                            </div>
                        `;
                    };
                    
                    // 更新模板项内容
                    templateItem.innerHTML = `
                        <div style="width: 100%; margin-bottom: 12px;">
                            <img src="${template.image_base64}" style="width: 100%; max-height: 300px; object-fit: contain; border-radius: 4px; background: #f8fafc; cursor: pointer;" 
                                 onclick="displayTemplateImage('${template.workpiece_id}', '${template.pose_id}')" />
                        </div>
                        
                        <div style="width: 100%; padding: 0 4px;">
                            <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; padding-bottom: 6px; border-bottom: 1px solid #e2e8f0;">
                                <div style="font-weight: 600; color: #1e293b; font-size: 0.95em;">模板: ${template.workpiece_id} / ${template.pose_id}</div>
                            </div>
                            
                            <div style="display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px;">
                                ${poseTypes.map(({ type, name, color, key }) => 
                                    createPoseBlock(poses[type], name, color, key)
                                ).join('')}
                            </div>
                        </div>
                    `;
                } catch (error) {
                    console.error('加载模板姿态数据失败:', error);
                    templateItem.innerHTML = `
                        <div style="width: 100%; margin-bottom: 12px;">
                            <img src="${template.image_base64}" style="width: 100%; max-height: 300px; object-fit: contain; border-radius: 4px; background: #f8fafc;" />
                        </div>
                        <div style="width: 100%; padding: 0 4px; color: #ef4444;">
                            加载姿态数据失败: ${error.message}
                        </div>
                    `;
                }
            });
            
            if (count === 0) {
                templateListContainer.innerHTML = '<div style="text-align: center; padding: 20px; color: #64748b;">该工件ID下暂无模板</div>';
            }
        } else {
            addLogEntry('error', `刷新模板列表失败: ${data.error}`);
            alert(`刷新模板列表失败: ${data.error}`);
        }
    })
    .catch(error => {
        addLogEntry('error', `刷新模板列表异常: ${error.message}`);
        console.error('刷新模板列表错误详情:', error);
        alert(`刷新模板列表异常: ${error.message}\n\n请检查：\n1. 服务器是否正常运行\n2. ROS2节点是否已启动`);
    });
}

// 显示模板图像（优先显示gripper_visualization.jpg）
function displayTemplateImage(workpieceId, poseId) {
    // 优先尝试加载gripper_visualization.jpg
    const imageUrl = `/api/get_template_image?workpiece_id=${encodeURIComponent(workpieceId)}&pose_id=${encodeURIComponent(poseId)}&image_name=gripper_visualization.jpg`;
    
    const placeholder = document.getElementById('template-image');
    placeholder.innerHTML = '<div style="text-align: center; padding: 20px; color: #64748b;">加载中...</div>';
    
    fetch(imageUrl)
    .then(response => {
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        return response.blob();
    })
    .then(blob => {
        const reader = new FileReader();
        reader.onload = function(e) {
            const imageBase64 = e.target.result;
            placeholder.innerHTML = '';
            const img = document.createElement('img');
            img.src = imageBase64;
            placeholder.appendChild(img);
            // 根据图像尺寸优化显示
            optimizeImageDisplay(img, placeholder);
            addLogEntry('success', `已显示模板图像: ${workpieceId}/pose_${poseId}/gripper_visualization.jpg`);
        };
        reader.readAsDataURL(blob);
    })
    .catch(error => {
        placeholder.innerHTML = '<div style="text-align: center; padding: 20px; color: #ef4444;">图像加载失败</div>';
        addLogEntry('error', `加载模板图像失败: ${error.message}`);
        console.error('加载模板图像错误:', error);
    });
}

// 刷新模板设置的工件ID下拉框
function refreshTemplateWorkpieceIdDropdown() {
    const dropdown = document.getElementById('workpiece-id');
    if (!dropdown) return;
    
    // 保存当前选中的值
    const currentValue = dropdown.value;
    
    // 调用API获取工件ID列表
    fetch(`${API_BASE_URL}/api/list_workpiece_ids`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    })
    .then(async response => {
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        return response.json();
    })
    .then(data => {
        if (data.success && data.workpiece_ids) {
            // 清空下拉框（保留第一个选项）
            dropdown.innerHTML = '<option value="">请选择或输入工件ID...</option>';
            
            // 填充选项
            data.workpiece_ids.sort().forEach(id => {
                const option = document.createElement('option');
                option.value = id;
                option.textContent = id;
                dropdown.appendChild(option);
            });
            
            // 恢复之前选中的值
            if (currentValue && Array.from(dropdown.options).some(opt => opt.value === currentValue)) {
                dropdown.value = currentValue;
            }
        }
    })
    .catch(error => {
        console.log('获取工件ID列表失败:', error);
    });
}

// 切换工件ID输入框显示
function toggleWorkpieceIdInput() {
    const select = document.getElementById('workpiece-id');
    const input = document.getElementById('workpiece-id-input');
    const container = input.parentElement;
    
    if (input.style.display === 'none') {
        select.style.display = 'none';
        input.style.display = 'block';
        input.value = select.value;
        // 确保输入框位置和大小与下拉框完全一致
        input.style.position = 'absolute';
        input.style.top = '0';
        input.style.left = '0';
        input.style.width = '100%';
        input.style.height = '38px';
        input.style.boxSizing = 'border-box';
        input.focus();
        input.select(); // 选中现有文本，方便直接输入
    } else {
        select.style.display = 'block';
        input.style.display = 'none';
    }
}

// 处理工件ID输入框失去焦点
function handleWorkpieceIdInputBlur() {
    const select = document.getElementById('workpiece-id');
    const input = document.getElementById('workpiece-id-input');
    const newValue = input.value.trim();
    
    if (newValue) {
        // 检查下拉框中是否已存在该值
        const existingOption = Array.from(select.options).find(opt => opt.value === newValue);
        if (!existingOption) {
            // 如果不存在，添加到下拉框
            const option = document.createElement('option');
            option.value = newValue;
            option.textContent = newValue;
            select.appendChild(option);
        }
        select.value = newValue;
    }
    
    select.style.display = 'block';
    input.style.display = 'none';
}

// 处理工件ID下拉框变化
function handleWorkpieceIdChange() {
    const select = document.getElementById('workpiece-id');
    const input = document.getElementById('workpiece-id-input');
    if (input.style.display !== 'none') {
        input.value = select.value;
    }
}

// 可视化抓取姿态
function visualizeGraspPose(templateImagePath, workpieceId, poseId) {
    addLogEntry('info', `可视化抓取姿态: 工件ID=${workpieceId}, 姿态ID=${poseId}`);
    
    // 调用后端API可视化抓取姿态
    fetch(`${API_BASE_URL}/api/visualize_grasp_pose`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            template_image_path: templateImagePath,
            workpiece_id: workpieceId,
            pose_id: poseId
        })
    })
    .then(async response => {
        if (!response.ok) {
            const text = await response.text();
            throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
        }
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            const text = await response.text();
            throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
        }
        return response.json();
    })
    .then(data => {
        if (data.success) {
            // 显示带夹爪可视化的图像
            displayTemplateImage(data.image_base64);
            addLogEntry('success', `抓取姿态可视化完成`);
        } else {
            addLogEntry('error', `可视化失败: ${data.error}`);
            alert(`可视化失败: ${data.error}`);
        }
    })
    .catch(error => {
        addLogEntry('error', `可视化异常: ${error.message}`);
        console.error('可视化错误详情:', error);
        alert(`可视化异常: ${error.message}\n\n请检查：\n1. 服务器是否正常运行\n2. ROS2节点是否已启动\n3. 标定文件是否存在`);
    });
}

// 模板设置选项卡的状态变量
let templateState = {
    currentImageBase64: null
};

function loadTemplateImage() {
    addLogEntry('info', '加载图像');
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = 'image/*';
    input.onchange = function(e) {
        const file = e.target.files[0];
        if (file) {
            const reader = new FileReader();
            reader.onload = function(e) {
                const imageBase64 = e.target.result;
                templateState.currentImageBase64 = imageBase64;
                
                const placeholder = document.getElementById('template-image');
                placeholder.innerHTML = '';
                const img = document.createElement('img');
                img.src = imageBase64;
                placeholder.appendChild(img);
                // 根据图像尺寸优化显示
                optimizeImageDisplay(img, placeholder);
                addLogEntry('success', '图像加载成功: ' + file.name);
            };
            reader.readAsDataURL(file);
        }
    };
    input.click();
}

function captureTemplateImage() {
    // 获取工件ID和姿态ID
    const workpieceId = document.getElementById('workpiece-id').value.trim();
    const poseId = document.getElementById('pose-id').value.trim();
    const cameraId = '207000152740'; // 默认相机ID
    
    if (!workpieceId || !poseId) {
        addLogEntry('warning', '请先输入工件ID和姿态ID');
        alert('请先输入工件ID和姿态ID');
        return;
    }
    
    addLogEntry('info', `正在触发相机拍照，相机ID: ${cameraId}，工件ID: ${workpieceId}，姿态ID: ${poseId}`);
    
    // 调用 capture_template_image 接口，会自动保存图像到对应的姿态文件夹
    fetch(`${API_BASE_URL}/api/capture_template_image`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            workpiece_id: workpieceId,
            pose_id: poseId,
            camera_id: cameraId
        })
    })
    .then(async response => {
        if (!response.ok) {
            const text = await response.text();
            throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
        }
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            const text = await response.text();
            throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
        }
        return response.json();
    })
    .then(data => {
        if (data.success) {
            // 获取JPEG格式的图像数据
            const imageBase64 = data.image_base64 || data.image;
            templateState.currentImageBase64 = imageBase64;
            
            // 显示图像到模板图像显示区域
            const placeholder = document.getElementById('template-image');
            placeholder.innerHTML = '';
            const img = document.createElement('img');
            img.src = imageBase64;
            placeholder.appendChild(img);
            // 根据图像尺寸优化显示
            optimizeImageDisplay(img, placeholder);
            
            // 显示保存路径信息（彩色图 + 深度图）
            const imagePath = data.image_path || `templates/${workpieceId}/pose_${poseId}/original_image.jpg`;
            addLogEntry('success', `图像采集完成，彩色图: ${imagePath}`);
            if (data.depth_image_path) {
                addLogEntry('success', `图像采集完成，深度图: ${data.depth_image_path}`);
            } else {
                addLogEntry('warning', '当前采集接口未返回深度图路径（可能未保存depth_image.png）');
            }
        } else {
            addLogEntry('error', '图像采集失败: ' + (data.error || '未知错误'));
            alert('图像采集失败: ' + (data.error || '未知错误'));
        }
    })
    .catch(error => {
        addLogEntry('error', '图像采集异常: ' + error.message);
        console.error('图像采集错误详情:', error);
        alert('图像采集异常: ' + error.message + '\n\n请检查：\n1. 服务器是否正常运行\n2. ROS2节点是否已启动\n3. 相机服务是否可用');
    });
}

function getRobotPose() {
    const workpieceId = document.getElementById('workpiece-id').value.trim();
    const poseId = document.getElementById('pose-id').value.trim();
    const poseType = document.getElementById('pose-type').value;
    
    if (!workpieceId || !poseId) {
        addLogEntry('warning', '请先输入工件ID和姿态ID');
        alert('请先输入工件ID和姿态ID');
        return;
    }
    
    const poseTypeNames = {
        'camera_pose': '相机姿态',
        'preparation_position': '准备姿态',
        'grab_position': '抓取姿态',
        'preplace_position': '预放置姿态',
        'place_position': '放置位姿'
    };
    
    addLogEntry('info', `正在获取${poseTypeNames[poseType]}: 工件ID=${workpieceId}, 姿态ID=${poseId}`);
    
    // 先获取机器人状态
    fetch(`${API_BASE_URL}/api/get_robot_status`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({})
    })
    .then(async response => {
        if (!response.ok) {
            const text = await response.text();
            throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
        }
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            const text = await response.text();
            throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
        }
        return response.json();
    })
    .then(data => {
        if (data.success && data.robot_status) {
            addLogEntry('success', `${poseTypeNames[poseType]}获取成功`);
            
            // 保存姿态到文件
            fetch(`${API_BASE_URL}/api/save_template_pose`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    workpiece_id: workpieceId,
                    pose_id: poseId,
                    pose_type: poseType,
                    robot_status: data.robot_status
                })
            })
            .then(async response => {
                if (!response.ok) {
                    const text = await response.text();
                    throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
                }
                const contentType = response.headers.get('content-type');
                if (!contentType || !contentType.includes('application/json')) {
                    const text = await response.text();
                    throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
                }
                return response.json();
            })
            .then(saveData => {
                if (saveData.success) {
                    // 显示姿态信息
                    const pos = data.robot_status.cartesian_position.position;
                    const ori = data.robot_status.cartesian_position.orientation;
                    const euler = data.robot_status.cartesian_position.euler_orientation_rpy_deg;
                    
                    const fmt = (v) => (v !== undefined && v !== null ? Number(v).toFixed(3) : '?');
                    const posStr = `位置(${fmt(pos.x)}, ${fmt(pos.y)}, ${fmt(pos.z)})m`;
                    const oriStr = `四元数(${fmt(ori.x)}, ${fmt(ori.y)}, ${fmt(ori.z)}, ${fmt(ori.w)})`;
                    const eulerStr = euler && euler.length >= 3 ? `, 姿态角(${euler[0].toFixed(2)}, ${euler[1].toFixed(2)}, ${euler[2].toFixed(2)})deg` : '';
                    
                    addLogEntry('success', `${poseTypeNames[poseType]}已保存到: ${saveData.file_path} - ${posStr}, ${oriStr}${eulerStr}`);
                    
                    const poseInfo = `位置: (${pos.x.toFixed(3)}, ${pos.y.toFixed(3)}, ${pos.z.toFixed(3)}) mm\n` +
                                   `姿态: (${euler[0].toFixed(2)}, ${euler[1].toFixed(2)}, ${euler[2].toFixed(2)}) deg`;
                    
                    alert(`${poseTypeNames[poseType]}获取并保存成功！\n\n${poseInfo}`);
    } else {
                    addLogEntry('error', `姿态保存失败: ${saveData.error}`);
                    alert(`姿态保存失败: ${saveData.error}`);
                }
            })
            .catch(error => {
                addLogEntry('error', `姿态保存异常: ${error.message}`);
                alert(`姿态保存异常: ${error.message}`);
            });
        } else {
            addLogEntry('error', `获取机器人状态失败: ${data.error || '未知错误'}`);
            alert(`获取机器人状态失败: ${data.error || '未知错误'}`);
        }
    })
    .catch(error => {
        addLogEntry('error', `获取姿态异常: ${error.message}`);
        alert(`获取姿态异常: ${error.message}`);
    });
}

function createTemplate() {
    const workpieceId = document.getElementById('workpiece-id').value.trim();
    const poseId = document.getElementById('pose-id').value.trim();
    
    if (!workpieceId || !poseId) {
        addLogEntry('warning', '请输入工件ID和姿态ID');
        alert('请输入工件ID和姿态ID');
        return;
    }
    
    addLogEntry('info', `检查模板完整性: 工件ID=${workpieceId}, 姿态ID=${poseId}`);
    
    // 检查模板文件是否存在
    const templatePath = `/home/nvidia/RVG_ws/templates/${workpieceId}/pose_${poseId}`;
    const requiredFiles = ['original_image.jpg', 'camera_pose.json', 
                          'preparation_position.json', 'grab_position.json'];
    
    let missingFiles = [];
    for (const file of requiredFiles) {
        // 可以添加文件检查逻辑
        // 暂时只显示提示
    }
    
    addLogEntry('info', `模板目录: ${templatePath}`);
    addLogEntry('success', `模板创建流程: 1.输入工件ID和姿态ID 2.采集图像 3.获取三种姿态`);
    alert('模板创建流程:\n1. 输入工件ID和姿态ID\n2. 点击"图像采集"按钮采集图像\n3. 选择姿态类型，点击"获取姿态"按钮获取并保存姿态\n4. 重复步骤3，获取所有姿态（相机姿态、准备姿态、抓取姿态、预放置姿态、放置位姿）');
}

function deleteTemplate() {
    addLogEntry('warning', '删除模板');
    // 模拟删除过程
    setTimeout(() => {
        addLogEntry('success', '模板删除完成');
        updateTemplateCount(1);
    }, 1000);
}

// 模板标准化功能
function standardizeTemplate() {
    const workpieceId = document.getElementById('workpiece-id').value.trim();
    
    if (!workpieceId) {
        addLogEntry('warning', '请输入工件ID');
        alert('请输入工件ID');
        return;
    }
    
    addLogEntry('info', `开始模板标准化: 工件ID=${workpieceId}`);
    
    fetch(`${API_BASE_URL}/api/standardize_template`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            workpiece_id: workpieceId
        })
    })
    .then(async response => {
        if (!response.ok) {
            const text = await response.text();
            throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
        }
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            const text = await response.text();
            throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
        }
        return response.json();
    })
    .then(data => {
        if (data.success) {
            addLogEntry('success', `模板标准化完成: 处理 ${data.processed_count} 个姿态, 跳过 ${data.skipped_count} 个姿态`);
            if (data.processed_pose_ids && data.processed_pose_ids.length > 0) {
                addLogEntry('info', `已处理的姿态: ${data.processed_pose_ids.join(', ')}`);
            }
            if (data.skipped_pose_ids && data.skipped_pose_ids.length > 0) {
                addLogEntry('warning', `跳过的姿态: ${data.skipped_pose_ids.join(', ')}`);
            }
            // 标准化完成后，刷新模板列表
            refreshTemplateList();
        } else {
            addLogEntry('error', `模板标准化失败: ${data.error_message}`);
            alert(`模板标准化失败: ${data.error_message}`);
        }
    })
    .catch(error => {
        addLogEntry('error', `模板标准化异常: ${error.message}`);
        console.error('模板标准化错误详情:', error);
        alert(`模板标准化异常: ${error.message}\n\n请检查：\n1. 服务器是否正常运行\n2. ROS2节点是否已启动\n3. 模板文件是否存在`);
    });
}

// 更新模板计数
function updateTemplateCount(count) {
    document.getElementById('template-count').textContent = count;
}

// Debug按钮功能
// 显示图像到Debug区域
function displayDebugImage(imageBase64, containerId) {
    const container = document.getElementById(containerId || 'debug-image-placeholder');
    if (!container) return;
    
    container.innerHTML = '';
    const img = document.createElement('img');
    img.src = imageBase64;
    container.appendChild(img);
    // 根据图像尺寸优化显示（Debug区域使用较小的padding）
    optimizeImageDisplay(img, container, { padding: 10 });
}

// 更新当前步骤显示
function updateDebugStepDisplay() {
    const stepElement = document.getElementById('debug-current-step');
    if (debugState.currentStep >= 0 && debugState.currentStep < debugState.steps.length) {
        stepElement.textContent = `${debugState.currentStep + 1}. ${debugState.steps[debugState.currentStep].name}`;
    } else {
        stepElement.textContent = '未开始';
    }
}

// 生成调试会话ID
function generateDebugSessionId() {
    const now = new Date();
    const timestamp = now.toISOString().replace(/[:.]/g, '-').slice(0, -5);
    return `debug_${timestamp}`;
}

function debugLoadImage() {
    addLogEntry('info', 'Debug: 加载图像');
    
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = 'image/*';
    input.onchange = function(e) {
        const file = e.target.files[0];
        if (file) {
            const reader = new FileReader();
            reader.onload = function(e) {
                const imageBase64 = e.target.result;
                debugState.originalImage = imageBase64;
                debugState.originalImageData = imageBase64;
                debugState.currentStep = 0;
                debugState.stepResults = {};
                debugState.debugSessionId = generateDebugSessionId();
                
                // 保存原始图像结果
                debugState.stepResults['original'] = {
                    image_base64: imageBase64,
                    metadata: {}
                };
                
                displayDebugImage(imageBase64);
                updateDebugStepDisplay();
                addLogEntry('success', `图像加载成功: ${file.name}`);
                addLogEntry('info', `调试会话ID: ${debugState.debugSessionId}`);
            };
            reader.readAsDataURL(file);
        }
    };
    input.click();
}

// ========== 新的 Debug 功能函数 ==========

function debugCaptureImage() {
    addLogEntry('info', 'Debug: 采集图像');
    
    const cameraId = '207000152740'; // 可以从输入框获取
    
    fetch(`${API_BASE_URL}/api/debug/capture`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            camera_id: cameraId
        })
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addLogEntry('success', '图像采集成功');
            // 立即刷新显示
            debugRefreshImages();
            // 启动自动刷新
            startDebugAutoRefresh();
        } else {
            // 采集失败：停止自动刷新，避免一直刷“等待图像”
            stopDebugAutoRefresh();
            addLogEntry('error', `图像采集失败: ${data.error}`);
            alert(`图像采集失败: ${data.error}`);
        }
    })
    .catch(error => {
        stopDebugAutoRefresh();
        addLogEntry('error', `图像采集异常: ${error.message}`);
        alert(`图像采集异常: ${error.message}`);
    });
}

function debugRefresh() {
    addLogEntry('info', 'Debug: 刷新显示');
    debugRefreshImages();
}

function debugRefreshImages() {
    if (debugState.isRefreshing) {
        return;
    }
    
    debugState.isRefreshing = true;
    
    fetch(`${API_BASE_URL}/api/debug/get_images`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success && data.has_images) {
            // 更新图像
            updateDebugImage('depth', data.depth_image, data.stats);
            updateDebugImage('color', data.color_image);
            updateDebugImage('binary', data.binary_image, data.stats);
            updateDebugImage('preprocessed', data.preprocessed_image, data.features);
        } else if (!data.has_images) {
            const waitMessage = data && data.message ? String(data.message) : '等待图像...';
            // 显示等待状态
            ['depth', 'color', 'binary', 'preprocessed'].forEach(type => {
                const content = document.getElementById(`${type}-image-content`);
                const stats = document.getElementById(`${type}-stats`);
                if (content) {
                    content.innerHTML = `
                        <div class="debug-image-placeholder">
                            <div class="placeholder-content">
                                <div class="placeholder-icon">📷</div>
                                <div class="placeholder-text">等待图像...</div>
                                <div class="placeholder-subtext">${waitMessage}</div>
                            </div>
                        </div>
                    `;
                }
                if (stats) {
                    stats.textContent = waitMessage;
                }
            });
        }
    })
    .catch(error => {
        console.error('刷新图像失败:', error);
    })
    .finally(() => {
        debugState.isRefreshing = false;
    });
}

function updateDebugImage(type, imageBase64, stats) {
    const content = document.getElementById(`${type}-image-content`);
    const statsElement = document.getElementById(`${type}-stats`);

    if (content && imageBase64) {
        content.innerHTML = '';
        const img = document.createElement('img');
        img.src = imageBase64;
        content.appendChild(img);
        // 根据图像尺寸优化显示（Debug区域使用较小的padding）
        optimizeImageDisplay(img, content, { padding: 10 });
    }
    
    let statsText = '';
    if (statsElement && stats) {
        let statsText = '';
        if (type === 'binary' && stats) {
            statsText = `轮廓: ${stats.filtered_contours}/${stats.total_contours} | 像素: ${stats.in_range_pixels}`;
        } else if (type === 'depth' && stats) {
            statsText = `深度范围: ${stats.min_depth || 0} - ${stats.max_depth || 0}`;
        }
        if (statsText) {
            statsElement.textContent = statsText;
        }
    }
    // 对于 color/preprocessed：即使传了 stats（例如 features），也可能没有可显示的 statsText。
    // 只要当前有图且没有生成 statsText，就清空初始的“等待图像...”
    if (statsElement && imageBase64 && !statsText) {
        statsElement.textContent = '';
    }
}

function startDebugAutoRefresh() {
    // 停止之前的定时器
    stopDebugAutoRefresh();
    
    // 启动新的定时器（每500ms刷新一次）
    debugState.refreshInterval = setInterval(() => {
        debugRefreshImages();
    }, 500);
    
    addLogEntry('info', 'Debug: 启动自动刷新');
}

function stopDebugAutoRefresh() {
    if (debugState.refreshInterval) {
        clearInterval(debugState.refreshInterval);
        debugState.refreshInterval = null;
        addLogEntry('info', 'Debug: 停止自动刷新');
    }
}

function nudgeParamSlider(sliderId, stepCount) {
    const slider = document.getElementById(sliderId);
    if (!slider) {
        return;
    }

    const min = slider.min !== '' ? parseFloat(slider.min) : -Infinity;
    const max = slider.max !== '' ? parseFloat(slider.max) : Infinity;
    // 微调按钮固定步长为 1（不使用滑动条的 step，便于精细调参）
    const step = 1;
    const originalStepAttr = slider.getAttribute('step');
    // 关键：range input 会按 step “吸附/校正” value。
    // 为了支持连续 +/-（2000->2001->2002...），一旦开始微调就把 step 固定切到 1，不立即恢复。
    if (!slider.dataset.nudgeOriginalStep) {
        slider.dataset.nudgeOriginalStep = originalStepAttr ?? '';
    }
    slider.step = '1';

    const cur = parseFloat(slider.value);
    let next = (Number.isFinite(cur) ? cur : 0) + stepCount * (Number.isFinite(step) ? step : 1);
    if (Number.isFinite(min)) next = Math.max(min, next);
    if (Number.isFinite(max)) next = Math.min(max, next);

    slider.value = String(next);

    // 复用滑动条的 inline oninput（对 aspect: this.value/10 非常关键）
    if (typeof slider.oninput === 'function') {
        slider.oninput();
    } else {
        slider.dispatchEvent(new Event('input', { bubbles: true }));
    }
}

// 长按微调（按下立即执行一次，随后连发；松开停止）
let nudgeHoldTimeout = null;
let nudgeHoldInterval = null;

function startNudge(sliderId, stepCount) {
    stopNudge();
    nudgeParamSlider(sliderId, stepCount);

    // 300ms 后开始连发
    nudgeHoldTimeout = setTimeout(() => {
        nudgeHoldInterval = setInterval(() => {
            nudgeParamSlider(sliderId, stepCount);
        }, 60);
    }, 300);
}

function stopNudge() {
    if (nudgeHoldTimeout) {
        clearTimeout(nudgeHoldTimeout);
        nudgeHoldTimeout = null;
    }
    if (nudgeHoldInterval) {
        clearInterval(nudgeHoldInterval);
        nudgeHoldInterval = null;
    }
}

function updateDebugParam(paramName, paramValue) {
    // 更新显示值
    const valueDisplay = document.getElementById(`${paramName}-value`);
    if (valueDisplay) {
        // 对于宽高比，显示原始值
        if (paramName.includes('aspect')) {
            valueDisplay.textContent = parseFloat(paramValue).toFixed(1);
        } else {
            valueDisplay.textContent = Math.round(paramValue);
        }
    }
    
    // 转换参数名（连字符转下划线）用于本地状态
    const serverParamName = paramName.replace(/-/g, '_');
    
    // 更新本地状态
    debugState.params[serverParamName] = parseFloat(paramValue);
    
    // 发送到服务器（保持连字符格式，后端支持这种格式）
    const requestBody = {
        param_name: paramName,  // 使用原始连字符格式
        param_value: parseFloat(paramValue)
    };
    
    fetch(`${API_BASE_URL}/api/debug/update_params`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            // 参数更新成功，刷新图像
            debugRefreshImages();
        }
    })
    .catch(error => {
        console.error('更新参数失败:', error);
    });
}

function debugSaveThresholds() {
    addLogEntry('info', 'Debug: 保存阈值');
    
    fetch(`${API_BASE_URL}/api/debug/save_thresholds`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success) {
            addLogEntry('success', `阈值保存成功: ${data.message}`);
            alert(`阈值保存成功: ${data.message}`);
        } else {
            addLogEntry('error', `阈值保存失败: ${data.error}`);
            alert(`阈值保存失败: ${data.error}`);
        }
    })
    .catch(error => {
        addLogEntry('error', `保存阈值异常: ${error.message}`);
        alert(`保存阈值异常: ${error.message}`);
    });
}

// 加载Debug参数
function loadDebugParams() {
    fetch(`${API_BASE_URL}/api/debug/get_params`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        }
    })
    .then(response => response.json())
    .then(data => {
        if (data.success && data.params) {
            console.log('[DEBUG] 加载参数:', data.params);
            
            // 参数名映射表：JSON中的参数名 -> HTML滑动条ID前缀
            const paramMap = {
                'binary_threshold_min': 'min-depth',
                'binary_threshold_max': 'max-depth',
                'component_min_area': 'contour-min-area',
                'component_max_area': 'contour-max-area',
                'component_min_aspect_ratio': 'min-aspect',
                'component_max_aspect_ratio': 'max-aspect',
                'component_min_width': 'min-width',
                'component_min_height': 'min-height',
                'component_max_count': 'max-count'
            };
            
            // 更新滑动条值和显示值
            let loadedCount = 0;
            let failedCount = 0;
            
            Object.keys(data.params).forEach(key => {
                const dashKey = paramMap[key];
                if (!dashKey) {
                    // 跳过不映射到滑动条的参数（如 enable_zero_interp）
                    return;
                }
                
                const sliderId = `${dashKey}-slider`;
                const valueId = `${dashKey}-value`;
                
                // 更新滑动条
                const slider = document.getElementById(sliderId);
                if (slider) {
                    let value = data.params[key];
                    
                    // 对于宽高比参数，需要乘以10
                    let sliderValue;
                    if (dashKey.includes('aspect')) {
                        sliderValue = Math.round(value * 10);
                    } else {
                        sliderValue = Math.round(value);
                    }
                    
                    // 确保值在滑动条的范围内
                    const minValue = parseFloat(slider.min);
                    const maxValue = parseFloat(slider.max);
                    
                    if (sliderValue < minValue) {
                        sliderValue = minValue;
                        console.warn(`[DEBUG] ${sliderId} 值 ${sliderValue} 小于最小值 ${minValue}，调整为 ${minValue}`);
                    } else if (sliderValue > maxValue) {
                        sliderValue = maxValue;
                        console.warn(`[DEBUG] ${sliderId} 值 ${sliderValue} 大于最大值 ${maxValue}，调整为 ${maxValue}`);
                    }
                    
                    // 临时将step设置为1，避免range input根据step属性自动调整值
                    // 这样可以精确设置任何值（如2095），而不是被吸附到step的倍数（如2100）
                    const originalStep = slider.getAttribute('step') || slider.step;
                    
                    // 保存原始step到dataset（nudgeParamSlider会使用）
                    if (originalStep && !slider.dataset.nudgeOriginalStep) {
                        slider.dataset.nudgeOriginalStep = originalStep;
                    }
                    
                    // 将step设置为1以精确设置值
                    slider.step = '1';
                    
                    // 设置滑动条的值（使用字符串确保正确设置）
                    slider.value = String(sliderValue);
                    
                    // 保持step为1，确保值不被吸附
                    // 这不会影响nudgeParamSlider的行为，因为它也会设置step为1
                    // 但对于拖拽，用户仍然可以通过滑动条调整，只是步长变为1
                    
                    console.log(`[DEBUG] 设置 ${sliderId} = ${slider.value} (来自 ${key})`);
                    
                    // 手动更新显示值（确保显示值正确）
                    // 注意：这里只更新显示，不触发oninput事件，避免在加载默认值时触发参数更新并覆盖配置文件
                    const valueDisplay = document.getElementById(valueId);
                    if (valueDisplay) {
                        let displayValue = value;
                        if (dashKey.includes('aspect')) {
                            displayValue = value.toFixed(1);
                        } else {
                            displayValue = Math.round(value);
                        }
                        valueDisplay.textContent = displayValue;
                    }
                    
                    // 不触发oninput事件，因为这只是加载配置文件中的默认值
                    // 不应该触发参数更新操作，避免覆盖配置文件
                    loadedCount++;
                } else {
                    console.warn(`[DEBUG] 未找到滑动条: ${sliderId} (参数: ${key})`);
                    failedCount++;
                }
            });
            
            // 更新debugState.params
            debugState.params = data.params;
            console.log('[DEBUG] 参数加载完成，当前参数:', debugState.params);
            addLogEntry('success', `Debug参数已从配置文件加载 (${loadedCount} 个滑动条已设置)`);
        } else {
            console.warn('[DEBUG] 未获取到有效参数');
            addLogEntry('warning', '未获取到有效参数，使用HTML默认值');
        }
    })
    .catch(error => {
        console.error('加载参数失败:', error);
        addLogEntry('warning', '加载参数失败，使用默认值');
    });
}

// 旧的函数保留用于兼容性
function debugRefreshImage() {
    debugRefresh();
}

function debugPrevious() {
    if (!debugState.originalImage) {
        addLogEntry('warning', '请先加载或采集图像');
        alert('请先加载或采集图像');
        return;
    }
    
    if (debugState.currentStep <= 0) {
        addLogEntry('warning', '已经在第一步');
        return;
    }
    
    debugState.currentStep--;
    updateDebugStepDisplay();
    
    // 显示上一步的结果
    if (debugState.currentStep === 0) {
        displayDebugImage(debugState.originalImageData);
        addLogEntry('info', '显示原始图像');
    } else {
        const stepId = debugState.steps[debugState.currentStep].id;
        if (debugState.stepResults[stepId]) {
            displayDebugImage(debugState.stepResults[stepId].image_base64);
            addLogEntry('info', `显示步骤: ${debugState.steps[debugState.currentStep].name}`);
        } else {
            // 如果结果不存在，调用API获取
            debugProcessStep(debugState.currentStep, false);
        }
    }
}

function debugRedo() {
    if (!debugState.originalImage) {
        addLogEntry('warning', '请先加载或采集图像');
        alert('请先加载或采集图像');
        return;
    }
    
    if (debugState.currentStep <= 0) {
        addLogEntry('warning', '当前是第一步，无法重做');
        return;
    }
    
    addLogEntry('info', `重做当前步骤: ${debugState.steps[debugState.currentStep].name}`);
    // 重新处理当前步骤
    debugProcessStep(debugState.currentStep, true);
}

function debugNext() {
    if (!debugState.originalImage) {
        addLogEntry('warning', '请先加载或采集图像');
        alert('请先加载或采集图像');
        return;
    }
    
    if (debugState.currentStep >= debugState.steps.length - 1) {
        addLogEntry('warning', '已经是最后一步');
        return;
    }
    
    debugState.currentStep++;
    updateDebugStepDisplay();
    
    // 如果是第一步（原始图像），直接显示
    if (debugState.currentStep === 0) {
        displayDebugImage(debugState.originalImageData);
        addLogEntry('info', '显示原始图像');
    } else {
        debugProcessStep(debugState.currentStep, false);
    }
}

function debugSaveResult() {
    if (!debugState.originalImage) {
        addLogEntry('warning', '请先加载或采集图像并完成特征提取');
        alert('请先加载或采集图像并完成特征提取');
        return;
    }
    
    addLogEntry('info', '保存特征提取结果...');
    
    // 收集所有特征数据
    const features = [];
    if (debugState.stepResults['features'] && debugState.stepResults['features'].features) {
        debugState.stepResults['features'].features.forEach((feature, index) => {
            features.push({
                id: index + 1,
                ub: feature.ub || 0,
                vb: feature.vb || 0,
                rb: feature.rb || 0,
                us: feature.us || 0,
                vs: feature.vs || 0,
                rs: feature.rs || 0,
                theta: feature.theta || 0,
                theta_deg: feature.theta_deg || 0,
                area: feature.area || 0
            });
        });
    }
    
    // 调用后端API保存CSV
    fetch(`${API_BASE_URL}/api/save_debug_features`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            session_id: debugState.debugSessionId,
            features: features,
            step: debugState.currentStep,
            step_name: debugState.steps[debugState.currentStep].name
        })
    })
    .then(async response => {
        if (!response.ok) {
            const text = await response.text();
            throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
        }
        return response.json();
    })
    .then(data => {
        if (data.success) {
            addLogEntry('success', `特征结果已保存: ${data.file_path}`);
            alert(`特征结果已保存到:\n${data.file_path}`);
        } else {
            addLogEntry('error', `保存失败: ${data.error}`);
            alert(`保存失败: ${data.error}`);
        }
    })
    .catch(error => {
        addLogEntry('error', `保存异常: ${error.message}`);
        alert(`保存异常: ${error.message}`);
    });
}

// 处理特征提取步骤
function debugProcessStep(stepIndex, isRedo) {
    if (!debugState.originalImage) {
        return;
    }
    
    // 如果是步骤0（原始图像），直接显示，不需要调用API
    if (stepIndex === 0) {
        displayDebugImage(debugState.originalImageData);
        addLogEntry('info', '显示原始图像');
        return;
    }
    
    const step = debugState.steps[stepIndex];
    addLogEntry('info', `处理步骤 ${stepIndex + 1}: ${step.name}`);
    
    // 获取输入图像
    let inputImage = debugState.originalImage;
    if (stepIndex > 0) {
        const prevStepId = debugState.steps[stepIndex - 1].id;
        if (debugState.stepResults[prevStepId]) {
            inputImage = debugState.stepResults[prevStepId].image_base64;
        } else if (prevStepId === 'original') {
            // 如果前一步是原始图像但结果不存在，使用原始图像
            inputImage = debugState.originalImage;
        }
    }
    
    // 调用后端API处理
    fetch(`${API_BASE_URL}/api/process_debug_step`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            session_id: debugState.debugSessionId,
            step_index: stepIndex,
            step_id: step.id,
            step_name: step.name,
            input_image: inputImage,
            is_redo: isRedo
        })
    })
    .then(async response => {
        if (!response.ok) {
            const text = await response.text();
            throw new Error(`HTTP ${response.status}: ${text.substring(0, 100)}`);
        }
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            const text = await response.text();
            throw new Error(`服务器返回的不是JSON格式: ${text.substring(0, 100)}`);
        }
        return response.json();
    })
    .then(data => {
        if (data.success) {
            // 保存结果
            debugState.stepResults[step.id] = {
                image_base64: data.result_image,
                metadata: data.metadata || {}
            };
            
            // 如果是特征提取步骤，保存特征数据
            if (step.id === 'features' && data.features) {
                debugState.stepResults[step.id].features = data.features;
                addLogEntry('info', `检测到 ${data.features.length} 个特征`);
            }
            
            // 显示结果
            displayDebugImage(data.result_image);
            addLogEntry('success', `步骤 ${stepIndex + 1} 处理完成: ${step.name}`);
            
            if (data.message) {
                addLogEntry('info', data.message);
            }
        } else {
            addLogEntry('error', `步骤处理失败: ${data.error}`);
            alert(`步骤处理失败: ${data.error}`);
        }
    })
    .catch(error => {
        addLogEntry('error', `步骤处理异常: ${error.message}`);
        alert(`步骤处理异常: ${error.message}`);
    });
}

// 添加日志条目
function addLogEntry(level, message) {
    const logContent = document.getElementById('log-content');
    const timestamp = new Date().toLocaleTimeString();
    
    const logEntry = document.createElement('div');
    logEntry.className = 'log-entry';
    logEntry.innerHTML = `
        <span class="log-timestamp">[${timestamp}]</span>
        <span class="log-level ${level}">[${level.toUpperCase()}]</span>
        <span>${message}</span>
    `;
    
    logContent.appendChild(logEntry);
    logContent.scrollTop = logContent.scrollHeight;
}

// 更新状态
function updateStatus(section, item, status) {
    // 这里可以添加更新状态指示器的逻辑
    console.log(`更新状态: ${section} - ${item} - ${status}`);
}

// 退出Web服务
function exitWebService() {
    // 显示确认对话框
    if (confirm('确定要退出Web服务吗？\n\n这将停止所有Web服务器进程。')) {
        addLogEntry('warning', '正在退出Web服务...');
        
        // 发送退出请求到服务器
        fetch(`${API_BASE_URL}/exit`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                action: 'exit_service'
            })
        })
        .then(response => {
            if (response.ok) {
                addLogEntry('success', 'Web服务已成功退出');
                // 延迟关闭页面
                setTimeout(() => {
                    window.close();
                }, 2000);
            } else {
                addLogEntry('error', '退出Web服务失败');
            }
        })
        .catch(error => {
            addLogEntry('error', '无法连接到服务器: ' + error.message);
            // 如果无法连接到服务器，直接关闭页面
            setTimeout(() => {
                window.close();
            }, 2000);
        });
    }
}

// 刷新工作流程的工件ID下拉框
function refreshWorkpieceIdDropdown() {
    const dropdown = document.getElementById('workpiece-id-workflow');
    if (!dropdown) return;
    
    // 保存当前选中的值
    const currentValue = dropdown.value;
    
    // 先尝试从list_templates API获取
    fetch(`${API_BASE_URL}/api/list_templates`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            templates_dir: ''
        })
    })
    .then(async response => {
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            throw new Error('服务器返回的不是JSON格式');
        }
        return response.json();
    })
    .then(data => {
        let workpieceIds = [];
        
        // 优先使用API返回的workpiece_ids字段（如果存在）
        if (data.success && data.workpiece_ids && data.workpiece_ids.length > 0) {
            workpieceIds = data.workpiece_ids;
        } 
        // 否则从模板列表中提取唯一的工件ID
        else if (data.success && data.templates && data.templates.length > 0) {
            workpieceIds = [...new Set(data.templates.map(t => t.workpiece_id).filter(id => id))];
        }
        
        // 如果从list_templates获取到了，直接使用
        if (workpieceIds.length > 0) {
            populateDropdown(workpieceIds, currentValue);
        } else {
            // 否则调用专门的API获取工件ID列表
            fetch(`${API_BASE_URL}/api/list_workpiece_ids`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            })
            .then(async response => {
                if (!response.ok) {
                    throw new Error(`HTTP ${response.status}`);
                }
                return response.json();
            })
            .then(data => {
                if (data.success && data.workpiece_ids) {
                    populateDropdown(data.workpiece_ids, currentValue);
                }
            })
            .catch(error => {
                console.log('获取工件ID列表失败:', error);
                addLogEntry('warning', '获取工件ID列表失败: ' + error.message);
            });
        }
    })
    .catch(error => {
        // 如果list_templates失败，直接调用list_workpiece_ids
        fetch(`${API_BASE_URL}/api/list_workpiece_ids`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            }
        })
        .then(async response => {
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}`);
            }
            return response.json();
        })
        .then(data => {
            if (data.success && data.workpiece_ids) {
                populateDropdown(data.workpiece_ids, currentValue);
            }
        })
        .catch(error2 => {
            console.log('获取工件ID列表失败:', error2);
            addLogEntry('warning', '获取工件ID列表失败: ' + error2.message);
        });
    });
    
    // 辅助函数：填充下拉框
    function populateDropdown(workpieceIds, currentValue) {
        const dropdown = document.getElementById('workpiece-id-workflow');
        if (!dropdown) return;
        
        // 清空下拉框（保留第一个选项）
        dropdown.innerHTML = '<option value="">请选择工件ID...</option>';
        
        // 填充选项
        workpieceIds.sort().forEach(id => {
            const option = document.createElement('option');
            option.value = id;
            option.textContent = id;
            dropdown.appendChild(option);
        });
        
        // 恢复之前选中的值
        if (currentValue && Array.from(dropdown.options).some(opt => opt.value === currentValue)) {
            dropdown.value = currentValue;
        }
        
        if (workpieceIds.length > 0) {
            addLogEntry('info', `已加载 ${workpieceIds.length} 个工件ID`);
        }
    }
}

// 自动填充工作流程的工件ID（已废弃，改用下拉框）
function autoFillWorkpieceId() {
    refreshWorkpieceIdDropdown();
}

// 刷新工作流程的姿态ID下拉框
function refreshPoseIdDropdown() {
    const workpieceId = document.getElementById('workpiece-id-workflow').value.trim();
    const poseIdDropdown = document.getElementById('pose-id-workflow');
    
    if (!poseIdDropdown) return;
    
    // 保存当前选中的值
    const currentValue = poseIdDropdown.value;
    
    if (!workpieceId) {
        // 如果没有选择工件ID，清空姿态ID下拉框
        poseIdDropdown.innerHTML = '<option value="">请先选择工件ID...</option>';
        return;
    }
    
    // 调用API获取该工件ID下的模板列表
    fetch(`${API_BASE_URL}/api/list_templates`, {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            templates_dir: '',
            workpiece_id: workpieceId
        })
    })
    .then(async response => {
        if (!response.ok) {
            throw new Error(`HTTP ${response.status}`);
        }
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
            throw new Error('服务器返回的不是JSON格式');
        }
        return response.json();
    })
    .then(data => {
        if (data.success && data.templates) {
            // 过滤出匹配的模板并提取唯一的姿态ID
            const filteredTemplates = data.templates.filter(t => t.workpiece_id === workpieceId);
            const poseIds = [...new Set(filteredTemplates.map(t => t.pose_id).filter(id => id))];
            
            // 按数字排序（如果可能）
            poseIds.sort((a, b) => {
                const numA = parseFloat(a);
                const numB = parseFloat(b);
                if (!isNaN(numA) && !isNaN(numB)) {
                    return numA - numB;
                }
                return String(a).localeCompare(String(b), undefined, { numeric: true, sensitivity: 'base' });
            });
            
            // 清空下拉框
            poseIdDropdown.innerHTML = '<option value="">请选择姿态ID...</option>';
            
            // 填充选项
            poseIds.forEach(id => {
                const option = document.createElement('option');
                option.value = id;
                option.textContent = id;
                poseIdDropdown.appendChild(option);
            });
            
            // 恢复之前选中的值（如果存在）
            if (currentValue && Array.from(poseIdDropdown.options).some(opt => opt.value === currentValue)) {
                poseIdDropdown.value = currentValue;
            } else if (poseIds.length > 0) {
                // 如果没有之前的值，默认选择第一个
                poseIdDropdown.value = poseIds[0];
            }
            
            if (poseIds.length > 0) {
                addLogEntry('info', `已加载 ${poseIds.length} 个姿态ID (工件ID: ${workpieceId})`);
            } else {
                addLogEntry('warning', `该工件ID下暂无模板 (工件ID: ${workpieceId})`);
            }
        } else {
            poseIdDropdown.innerHTML = '<option value="">未找到模板...</option>';
        }
    })
    .catch(error => {
        console.log('获取姿态ID列表失败:', error);
        addLogEntry('warning', '获取姿态ID列表失败: ' + error.message);
        poseIdDropdown.innerHTML = '<option value="">获取失败...</option>';
    });
}

// 页面加载完成后的初始化
document.addEventListener('DOMContentLoaded', function() {
    addLogEntry('success', 'Web界面加载完成');
    
    // 刷新工作流程的工件ID下拉框
    refreshWorkpieceIdDropdown();
    
    // 加载Debug参数（从JSON文件初始化滑动条）
    loadDebugParams();
    
});

// 窗口大小改变时的自适应处理
window.addEventListener('resize', function() {
    // 这里可以添加响应式处理的逻辑
    // console.log('窗口大小改变: ' + window.innerWidth + 'x' + window.innerHeight);
});
