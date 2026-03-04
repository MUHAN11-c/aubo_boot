// 最简单的测试脚本
console.log('=== 简单测试脚本开始执行 ===');

window.testFunction = function() {
    return '测试成功';
};

window.testVariable = 'Hello World';

console.log('=== 简单测试脚本执行完成 ===');
console.log('testFunction存在:', typeof window.testFunction);
console.log('testVariable存在:', typeof window.testVariable);





