# SCARA LU Planner 语法与设计笔记

## 1. `std::atomic_bool _running{false};` 是什么

### 定义
`_running` 是一个**原子布尔变量**，表示：

- 当前规划器是否处于“运行/发布”状态

典型声明：

```cpp
std::atomic_bool _running{false};
```

### 为什么不用普通 `bool`
因为它会被**多个回调/线程同时访问**：

- OPC UA 的 `start/stop` 方法会写它
- 定时器回调会读它

如果用普通 `bool`，可能出现**数据竞争**。  
用 `std::atomic_bool` 可以保证读写是安全的。

---

## 2. `_running.store(...)` 和 `_running.load(...)` 是什么意思

### `store`
表示：**写入一个值**

例如：

```cpp
_running.store(true);
```

意思是把 `_running` 设为 `true`。

### `load`
表示：**读取当前值**

例如：

```cpp
if (_running.load()) {
    // 执行发布
}
```

意思是读取 `_running` 当前是不是 `true`。

---

## 3. 这份代码里 `_running` 的整体逻辑

### Start 时
典型逻辑：

```cpp
bool ok = enableCspFollow();
_running.store(ok);
```

含义：

- 先尝试切到 CSP 跟随模式
- 如果成功，`ok == true`
- 再把 `_running` 设为 `true`
- 之后定时器开始发布目标

即：

- Start 成功 → `_running = true`
- Start 失败 → `_running = false`

---

### Stop 时
典型逻辑：

```cpp
_running.store(false);
bool ok = disableFollow();
```

含义：

- 先立刻停止发布
- 再退出 follow 模式

这样更安全，因为先把“发命令开关”关掉了。

---

### Timer 时
典型逻辑：

```cpp
if (!_running.load()) return;
```

含义：

- 如果没在运行，就什么也不发
- 如果在运行，才继续发布轨迹点

---

## 4. 为什么函数前四个参数不是引用，最后一个才是引用

函数：

```cpp
bool cartToq(double x, double y, double z, double r_deg, std::array<double, 4> &q_out)
```

### 前四个参数为什么按值传递
前四个参数都是 `double`：

- 类型很小
- 拷贝代价低
- 按值传递最常见
- 写法简单清晰

所以通常写成：

```cpp
double x, double y, double z, double r_deg
```

而不是：

```cpp
const double& x
```

---

### 最后一个参数为什么是引用
最后一个参数 `q_out` 是**输出参数**。

函数内部会给它赋值：

```cpp
q_out = {qv[0], qv[1], qv[2], qv[3]};
```

如果它不是引用，而是按值传递，那么修改的只是副本，调用者拿不到结果。

所以这里用：

```cpp
std::array<double, 4> &q_out
```

表示：

- 调用者提供一个数组
- 函数把计算结果写进去

---

## 5. 为什么 server 是对象，而 client 是引用

头文件中：

```cpp
rm::OpcuaServer _srv;
rm::OpcuaClient &_exec;
```

### `_srv` 为什么是对象
因为 `_srv` 是当前类**自己创建并拥有**的。

构造函数里直接构造：

```cpp
: _srv(cfg::kCtrlPort, "SCARA LoadUnload Planner")
```

说明：

- 服务器由 `ScaraLUPlanner` 自己管理
- 生命周期跟当前对象一致
- 这是“拥有关系”

所以写成成员对象：

```cpp
rm::OpcuaServer _srv;
```

---

### `_exec` 为什么是引用
因为 `_exec` 是**外部传进来的现成 client**：

```cpp
explicit ScaraLUPlanner(std::string_view name, rm::OpcuaClient &exec);
```

说明：

- `ScaraLUPlanner` 不负责创建它
- 只是借用它
- 外部对象必须先存在

所以这里是“借用关系”，适合写引用：

```cpp
rm::OpcuaClient &_exec;
```

---

### 一句话总结
- 对象：**自己拥有**
- 引用：**借用外部对象**

---

## 6. 终端报错 `shadows a parameter` 是什么意思

报错示例：

```text
error: declaration of ‘auto q_out’ shadows a parameter
```

函数参数里本来已经有一个 `q_out`：

```cpp
bool ScaraLUPlanner::cartToq(..., std::array<double, 4> &q_out)
```

但函数内部又写了：

```cpp
auto q_out = robomotion.motorinfo.pulsesToUnit(p);
```

这就等于又声明了一个同名变量，把原来的参数名**遮住了**。

这叫：

- shadow
- 遮蔽
- 同名局部变量遮住形参

---

### 正确写法
把局部变量改名即可，例如：

```cpp
auto qv = robomotion.motorinfo.pulsesToUnit(p);
```

然后：

```cpp
q_out = {qv[0], qv[1], qv[2], qv[3]};
```

---

## 7. 输出参数的典型识别方法

看到这种函数签名时：

```cpp
bool func(..., T &out)
```

通常表示：

- 返回值 `bool`：表示成功或失败
- `out`：保存真正计算出的结果

例如：

```cpp
bool cartToq(..., std::array<double, 4> &q_out)
```

含义就是：

- IK 成功则返回 `true`，并把结果写到 `q_out`
- IK 失败则返回 `false`

---

## 8. 这类知识点应该怎么记

建议分两层记录：

### 代码里只写短注释
例如：

```cpp
std::atomic_bool _running{false}; //!< 是否允许发布轨迹
rm::OpcuaClient &_exec;           //!< 外部传入的客户端引用
```

### 详细理解写到 Markdown 笔记
适合记录：

- 是什么
- 为什么这样设计
- 在代码哪几处使用
- 改错会有什么后果

---

## 9. 快速复习版

### `_running`
- 类型：`std::atomic_bool`
- 含义：运行开关
- Start：成功则 `store(true)`
- Stop：`store(false)`
- Timer：`load()` 决定是否发布

### `cartToq(...)`
- 前四个 `double`：输入参数，按值传递
- `q_out`：输出参数，用引用传递

### `_srv` 与 `_exec`
- `_srv`：自己拥有，所以是对象
- `_exec`：借用外部对象，所以是引用

### `shadow parameter`
- 形参与局部变量重名
- 局部变量把参数名遮住
- 改局部变量名即可

---

## 10. 一句原则

### C++ 中常见判断方法
- 小标量输入：按值传递
- 输出结果：引用参数
- 自己拥有的成员：对象
- 借用外部资源：引用或指针
- 多回调共享状态：优先考虑 `atomic` 或锁