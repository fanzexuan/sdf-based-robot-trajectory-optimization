# 基于连续隐式 SDF 的任意形状机器人轨迹优化

首先这篇文章禁止转载，主要是如果有对这篇论文的不同见解方便反馈，我的邮箱（fanzexuan135@163.com)。
正文开始：这篇论文《Continuous Implicit SDF Based
Any-shape Robot Trajectory Optimization》引起了我极大的兴趣，任意形状可以进行高精度规划（实际规避障碍物的效果非常好，但是轨迹合理性，比如可以走更宽敞的地方避免在窄区域闪转腾挪，包括最优性应该也还有优化空间，当然这不是本论文的重点），下面我对这篇论文的理论和不好理解的地方做下个人理解的阐述。   
Github中markdown不能正确显示，可以在repo中下载html文件查看；

## 1. 引言

在机器人的运动规划中，常用的几何表示和计算方法包括：

1. 用简单几何体（如椭球体、多面体等）近似机器人的形状。这种方法精度不高，导致规划出的轨迹过于保守。
2. 用采样点云表示机器人的表面。这种方法受限于采样密度，在低分辨率下可能遗漏碰撞，高分辨率下表示又过于复杂。

对于环境表示，优化类的轨迹规划方法通常需要预先计算并存储环境的 SDF（signed distance field）或安全通道等信息，这带来了额外的计算和存储开销。

总结现有方法存在两个问题：
1. 很难高效准确地对任意形状的机器人建模；
2. 轨迹优化需要环境的额外信息如 SDF 等。

本文的关键思想是：**任意机器人的表面可以用其 SDF 的零等值面隐式表示**。基于此，作者进一步利用一个隐函数来简化的计算机器人沿轨迹扫出体积（swept volume）的 SDF。通过利用时空连续性，该计算是高效的，并且隐式表示保证了对任意形状机器人的精确连续碰撞检测。此外，作者还提出了一个基于隐式 SDF 的轨迹优化方法。

## 2. 机器人的隐式 SDF 表示

作者用一个隐式连续函数 $SDF_B: \mathbb{R}^3 \to \mathbb{R}$ 来表示任意形状的机器人 $B$，该函数在机器人内部为负值。三角网格是表示任意形状的通用成熟方法。文中用 winding number signed distance field [20] 来实现隐式 SDF。利用现有算法和库如 LIBIGL，可以高效地计算任意查询点 $\boldsymbol{x}$ 处的 $SDF_B(\boldsymbol{x})$ 值及其梯度 $\nabla SDF_B|_{\boldsymbol{x}}$。

## 3. 扫出体积的隐式 SDF 表示

随着机器人运动，其 SDF 变成时变函数：

$$
f_{sdf}(\boldsymbol{x}_{ob}, t) = SDF_{B(t)}(\boldsymbol{x}_{ob}) = SDF_{R(t)B+\boldsymbol{p}(t)}(\boldsymbol{x}_{ob})
$$

根据运动的相对性，该函数可改写为：

$$
f_{sdf}(\boldsymbol{x}_{ob}, t) = SDF_B\big(R^{-1}(t)(\boldsymbol{x}_{ob}-\boldsymbol{p}(t))\big)
$$

其中 $R(t), \boldsymbol{p}(t)$ 分别是机器人的姿态和位置。

直观上，对任意查询点 $\boldsymbol{x}_{ob}$，如果 $f_{sdf}(\boldsymbol{x}_{ob}, t)$ 在时域上取最小值，对应的时刻 $t^*$ 就是 $\boldsymbol{x}_{ob}$ 到扫出体积（swept volume）的距离。假设 $\boldsymbol{p}(t), R(t)$ 连续，则 $f_{sdf}(\boldsymbol{x}_{ob}, t)$ 在时空上连续，最小值 $f^*_{sdf}$ 容易通过数值方法求得。

## 4. 基于时空连续的高效 SDF 计算

文中利用梯度下降和 Armijo 线搜索来计算 $t^* = \arg\min_t f_{sdf}(\boldsymbol{x}_{ob}, t)$。收敛速度取决于初值选择。作者在轨迹上均匀采样，找到 $\boldsymbol{x}_{ob}$ 到采样机器人位置的最近点作为 $t_{init}$。对于 $\boldsymbol{x}_{ob}$ 附近的其他查询点，则用 $\boldsymbol{x}_{ob}$ 的 $t^*$ 值初始化，加速了计算。每次查询只需微秒级时间。

## 5. 基于优化的轨迹生成

以四旋翼无人机为例，其微分平坦性使得姿态轨迹可由位置轨迹唯一确定，降低了优化问题的维度。作者采用了一种最小控制力多项式（MINCO）的轨迹表示方法（MINCO是浙大这个课题组自己使用的一种模型定义，可以看浙大FAST实验室发表的相关论文）。

将轨迹生成构建为一个无约束优化问题：

$$
\min_{\boldsymbol{c}, \boldsymbol{T}} \quad  \lambda_s J_s + \lambda_m J_m + \lambda_d J_d + \rho J_t
$$

其中 $J_s, J_m, J_d, J_t$ 分别是安全性、平滑性、动力学可行性和总时间代价项。

传统方法中安全项 $J_s$ 通常是沿轨迹积分得到：

$$
J_s = \int_{t_{min}}^{t_{max}} J_s(\boldsymbol{c}, \boldsymbol{T}, t) \, dt
$$

由于本文没有 $f_{sdf}$ 的解析形式，直接积分存在遗漏碰撞的风险，且在稀疏环境中许多采样点是冗余的。相比之下，该论文的方法无需沿轨迹采样，只需对障碍物点求 $f^*_{sdf}$，理论上避免了这些问题，效率也更高。

此外，文章还给出了各项的梯度解析形式，并提出了一种隐式求解 $t^*$ 对各优化变量梯度的方法。

## 小结
本文利用隐函数将机器人和其轨迹扫出体积统一表示，基于时空连续性快速计算任意点到 swept volume 的距离，用于构建优化问题。该方法适用于任意形状的机器人，可实现精确连续避障。

### 代码事例（非完整）
```python
# 加载机器人表面三角网格
V, F = load_mesh("robot.obj") 

# 定义 MINCO 轨迹
p = MINCO_traj(q, T) 

# 隐式 SDF 查询函数
def query_f_sdf(x_ob, t):
    R, p = p.rotation(t), p.translation(t)
    return signed_distance(V, F, np.dot(R.T, x_ob - p))

def swept_volume_sdf(x_ob):  
    t_min, t_max = 0, p.duration
    t_star = scipy.optimize.minimize(lambda t: query_f_sdf(x_ob, t), 
                                     x0=t_init, bounds=[(t_min, t_max)]).x
    f_star_sdf = query_f_sdf(x_ob, t_star)
    return f_star_sdf, t_star

# 构建优化问题
def optimize(q_init, T_init):
    q, T = q_init, T_init
    for i in range(max_iters):
        J_s, gradq_J_s, gradT_J_s = eval_safety_cost(q, T)  
        J_m, gradq_J_m, gradT_J_m = eval_smooth_cost(q, T)
        ...
        
        # 求各代价项梯度,用于更新 q, T
        gradq_J = λ_s*gradq_J_s + λ_m*gradq_J_m + ...
        gradT_J = λ_s*gradT_J_s + λ_m*gradT_J_m + ...        
        q = q - lr * gradq_J
        T = T - lr * gradT_J
```

关键是隐式 SDF 的查询函数 `swept_volume_sdf`。它只需要机器人表面三角网格的顶点 `V` 和拓扑 `F` 信息。通过数值优化找到任意 $\boldsymbol{x}_{ob}$ 处的 $t^*$ 值，代入 `query_f_sdf` 即可得到 $f^*_{sdf}$。

这种隐式表示使得该方法与具体的机器人形状无关，可以灵活应用于各种机器人。同时基于时空连续性的快速 SDF 计算，也为优化问题提供了精确、连续的碰撞约束信息。该方法为任意形状机器人的轨迹优化提供了一个统一、高效的框架。


## Q&A:
##### 1.隐函数(Implicit Function) 与 KKT条件
在数学中,隐函数(Implicit Function)是指一个多元函数f(x1,x2,...,xn)=0所定义的函数关系,其中任一变量都没有被显式表示为其他变量的函数。这与显函数(Explicit Function)相对,后者给出了因变量与自变量之间的显式表达式如y=f(x)。

本文中使用隐式表示有以下几个原因:

1. 通用性:机器人的形状千变万化,用显式函数(如球、椭球、多面体等)很难统一描述。而隐函数可以灵活表示任意形状的零等值面。例如球面可以用隐式表示 $f(x,y,z)=x^2+y^2+z^2-R^2=0$,而不必用显式的参数方程。

2. 封闭性:机器人为封闭的物体,其内外可用SDF的正负符号区分。隐式SDF函数自然满足这一性质。若机器人表面为 $f(x,y,z)=0$,则可定义其SDF为 $SDF(x,y,z)=\begin{cases} -d((x,y,z),\partial B), & (x,y,z)\in B^{\circ} \\ 0, & (x,y,z)\in \partial B \\ +d((x,y,z),\partial B), & (x,y,z)\in \mathbb{R}^3\setminus B \end{cases}$ ,其中 $B, \partial B, B^{\circ}$ 分别为机器人、表面和内部, $d$ 为欧氏距离。

3. 连续性:本文利用机器人SDF关于时空的连续性,高效计算轨迹扫出体积的SDF。对于连续运动,SDF在时空上也是连续变化的,这保证了数值方法的可行性。显式表示则可能在某些奇异位置不连续。

此外,隐式表示也便于求解SDF的梯度。例如对隐函数 $f(x,y,z)=0$ 求SDF实际是一个有约束优化问题:

$$
\min_{(x,y,z)} \sqrt{(x-x_0)^2+(y-y_0)^2+(z-z_0)^2} \quad s.t. \quad f(x,y,z)=0
$$

其KKT条件给出了解析梯度(省略约束违反项):

$$
\nabla SDF(x_0,y_0,z_0) = \frac{(x^*-x_0,y^*-y_0,z^*-z_0)}{\sqrt{(x^*-x_0)^2+(y^*-y_0)^2+(z^*-z_0)^2}}
$$

其中 $(x^*,y^*,z^*)$ 是最近点,可用数值方法求得。

综上,隐式SDF能以统一的方式连续表示任意形状,并便于梯度求解,是一种强大的几何表示。将其引入轨迹优化领域,可以提高规划的通用性和鲁棒性。本文很好地体现了这一点。  

KKT条件,全称Karush-Kuhn-Tucker条件,是非线性规划中一个重要的最优性条件。它给出了一个约束优化问题的最优解所必须满足的一阶必要条件。在满足一定的规则性条件下,KKT条件也是充分条件。

考虑以下的约束优化问题:

$$
\begin{aligned}
\min_{x} \quad & f(x) \\
\text{s.t.} \quad & g_i(x) \leq 0, \quad i=1,\ldots,m \\
& h_j(x) = 0, \quad j=1,\ldots,p
\end{aligned}
$$

其中$f(x)$是目标函数,$g_i(x)$是不等式约束,$h_j(x)$是等式约束。假设这些函数都是连续可微的。

令$x^*$是此问题的一个局部最优解,则在$x^*$点处,存在常数$\mu_i^* \geq 0$和$\lambda_j^*$,使得以下条件成立:

1. Stationarity: $\nabla f(x^*) + \sum_{i=1}^{m} \mu_i^* \nabla g_i(x^*) + \sum_{j=1}^{p} \lambda_j^* \nabla h_j(x^*) = 0$
2. Primal feasibility: $g_i(x^*) \leq 0, \quad i=1,\ldots,m; \quad h_j(x^*) = 0, \quad j=1,\ldots,p$  
3. Dual feasibility: $\mu_i^* \geq 0, \quad i=1,\ldots,m$
4. Complementary slackness: $\mu_i^* g_i(x^*) = 0, \quad i=1,\ldots,m$

这里$\mu_i^*$和$\lambda_j^*$被称为KKT乘子或拉格朗日乘子。它们的物理意义是最优点处约束对目标函数的敏感度。

KKT条件的重要性在于:
1. 它将约束优化问题转化为一个等式方程组,便于数值求解。很多优化算法如SQP、内点法等都是基于KKT条件设计的。
2. 它给出了最优解处目标函数和约束的梯度之间的关系,有助于敏感度分析和优化问题的构建。
3. 对偶可行性条件揭示了原问题与对偶问题之间的联系,这是凸优化中的重要理论。

在本文中,作者利用KKT条件巧妙地推导出了隐式表面SDF的解析梯度公式,避免了数值微分的昂贵开销。这充分体现了KKT条件作为优化理论在实际问题中的有力应用。优化在机器人领域中无处不在,KKT条件可以说是优化的基石,值得每一位机器人研究者深入理解和掌握。   


基于时空连续的高效 SDF 计算是本文的一个核心内容,目的是快速计算任意查询点到机器人轨迹扫掠体积（swept volume）的 SDF。这个过程可以分为两步：

1. 对于查询点 $\boldsymbol{x}_{ob}$，找到其对应的最近时刻 $t^*$，即 $f_{sdf}(\boldsymbol{x}_{ob}, t)$ 取最小值时的 $t$。
2. 将 $t^*$ 代入 $f_{sdf}(\boldsymbol{x}_{ob}, t^*)$ 得到 $\boldsymbol{x}_{ob}$ 到 swept volume 的 SDF 值 $f^*_{sdf}$。

第一步是一个单变量无约束优化问题：

$$
t^* = \arg\min_t f_{sdf}(\boldsymbol{x}_{ob}, t)
$$

为了高效求解，文中使用了梯度下降法，并用 Armijo 线搜索来确定步长。下面详细解释这两个方法：

###### 梯度下降法
梯度下降法是一种一阶优化算法，通过不断沿负梯度方向更新变量以达到最小值点。对于上述问题，迭代公式为：

$$
t_{k+1} = t_k - \alpha_k \nabla_t f_{sdf}(\boldsymbol{x}_{ob}, t_k)
$$

其中 $\alpha_k$ 是第 $k$ 步的步长，通常由线搜索确定。$\nabla_t f_{sdf}$ 是 $f_{sdf}$ 关于 $t$ 的梯度，文中给出了它的解析表达式（这里不再赘述）。

梯度下降法简单有效，但可能收敛到局部最优。为了得到全局最优，文中使用了多点初始化策略：
- 在轨迹上均匀采样，找到 $\boldsymbol{x}_{ob}$ 在采样时刻的最近点，作为 $t$ 的初值 $t_{init}$。
- 对于 $\boldsymbol{x}_{ob}$ 附近的查询点，用已计算出的 $t^*$ 值作为初值。

这样可以提高全局收敛性，又利用了 $t^*$ 在空间上的连续性来加速计算。

###### Armijo 线搜索
在梯度下降中，步长 $\alpha_k$ 的选择至关重要。太小会导致收敛缓慢，太大则可能越过最小值点。Armijo 线搜索是一种自适应步长策略，它以保证函数充分下降为目标，从一个初始步长（比如1）开始，不断缩小步长直到满足 Armijo 条件：

$$
f_{sdf}(\boldsymbol{x}_{ob}, t_k - \alpha_k \nabla_t f_{sdf}(\boldsymbol{x}_{ob}, t_k)) \leq 
f_{sdf}(\boldsymbol{x}_{ob}, t_k) - c \alpha_k \|\nabla_t f_{sdf}(\boldsymbol{x}_{ob}, t_k)\|^2
$$

其中 $c\in(0,1)$ 是一个常数参数，通常取 $c=0.1$ 或更小。这个条件保证了每一步都能显著降低函数值。Armijo 搜索通过反复试探步长，在降低函数值和缩短搜索时间之间取得平衡。

综合使用梯度下降和 Armijo 线搜索，可以高效稳定地找到 $t^*$。本文的实验表明，每次查询只需要微秒级的时间，速度很快。这得益于以下几点：
1. $t^*$ 和 $f^*_{sdf}$ 具有时空连续性，易于数值求解。
2. $f_{sdf}$ 的解析梯度避免了数值微分的开销。
3. 多点初始化策略提高了收敛速度和全局最优性。

这部分计算是整个轨迹优化的基础，它提供了快速、精确的碰撞检测信息。基于此，文章进一步构建了一个梯度下降的优化框架，最小化各种损失函数（如安全性、平滑性等），生成一条最优轨迹。从这个意义上说，高效 SDF 计算是本文的一个关键环节，它体现了隐式表示和时空连续性在轨迹优化中的巧妙应用。 


##### 2.微分平坦性(Differential Flatness)
微分平坦性(Differential Flatness)是控制理论中的一个重要概念,主要用于简化非线性系统的轨迹生成和控制问题。一个系统被称为微分平坦的,如果它的所有状态变量和控制输入都可以用一组特殊变量(称为平坦输出)及其导数来表示。

形式化地说,考虑一个具有状态变量 $\boldsymbol{x}\in\mathbb{R}^n$ 和控制输入 $\boldsymbol{u}\in\mathbb{R}^m$ 的非线性系统:

$$
\dot{\boldsymbol{x}} = f(\boldsymbol{x}, \boldsymbol{u})
$$

如果存在一组变量 $\boldsymbol{z}\in\mathbb{R}^m$ (平坦输出),使得状态和控制输入都可以表示为 $\boldsymbol{z}$ 及其导数的函数:

$$
\begin{aligned}
\boldsymbol{x} &= \phi(\boldsymbol{z}, \dot{\boldsymbol{z}}, \ldots, \boldsymbol{z}^{(p)}) \\
\boldsymbol{u} &= \psi(\boldsymbol{z}, \dot{\boldsymbol{z}}, \ldots, \boldsymbol{z}^{(q)})
\end{aligned}
$$

其中 $p,q$ 是有限整数,则称该系统是微分平坦的,而 $\boldsymbol{z}$ 就是该系统的平坦输出。

微分平坦性的重要性在于:
1. 它将非线性系统的状态空间轨迹规划问题转化为平坦输出空间的轨迹规划问题。后者通常是一个低维空间,约束条件更少,因此大大简化了轨迹生成任务。
2. 在平坦输出空间设计的轨迹可以通过微分平坦性映射回原状态空间,保证动力学可行性。这避免了在原空间进行复杂的约束优化。
3. 它为非线性系统提供了一种系统的控制方法。只需在平坦输出空间设计简单的控制律(如PID),然后通过微分平坦性变换即可得到原系统的控制输入。

用一种更直观的方式解释微分平坦性。

举个例子来说，假设有一个机器人,它的状态由位置 $x$,速度 $v$,加速度 $a$ 组成。通常,你需要知道这三个量才能完全描述机器人的运动状态。但如果这个机器人满足微分平坦性,那么你只需要知道其中一个量,比如位置 $x$,就可以推导出其他两个量。

这是因为,如果系统是微分平坦的,那么所有状态变量和控制输入都可以写成某个特殊变量（平坦输出）及其导数的函数。在这个例子中,如果位置 $x$ 是平坦输出,那么:

$$
\begin{aligned}
v &= \dot{x} \\
a &= \ddot{x}
\end{aligned}
$$

这意味着,一旦我们知道了位置 $x$ 随时间的函数关系,就自动知道了速度 $v$ 和加速度 $a$。这个性质在轨迹规划中非常有用。

假设我们要让机器人从起点移动到终点。如果不考虑微分平坦性,我们需要同时规划位置、速度、加速度的轨迹,并确保它们之间满足微分关系,这可能很复杂。但如果系统是微分平坦的,我们只需要规划一条满足起点和终点约束的位置轨迹 $x(t)$,然后自动得到速度轨迹 $v(t)=\dot{x}(t)$ 和加速度轨迹 $a(t)=\ddot{x}(t)$,大大简化了问题。

对于四旋翼无人机,情况类似但更复杂一些。它的平坦输出是位置 $\boldsymbol{p}$ 和偏航角 $\psi$。给定 $\boldsymbol{p}(t)$ 和 $\psi(t)$,我们可以推导出速度 $\boldsymbol{v}(t)$,加速度 $\boldsymbol{a}(t)$,姿态角 $\boldsymbol{\Omega}(t)$ 和姿态角速度 $\dot{\boldsymbol{\Omega}}(t)$。这就是论文中的那个公式:

$$
\begin{aligned}
\boldsymbol{v}(t) &= \dot{\boldsymbol{p}}(t) \\
\boldsymbol{a}(t) &= \ddot{\boldsymbol{p}}(t) \\
\boldsymbol{\Omega}(t) &= f_1(\boldsymbol{p}, \dot{\boldsymbol{p}}, \ddot{\boldsymbol{p}}, \psi) \\
\dot{\boldsymbol{\Omega}}(t) &= f_2(\boldsymbol{p}, \dot{\boldsymbol{p}}, \ddot{\boldsymbol{p}}, \dot{}\ddot{\boldsymbol{p}}, \psi, \dot{\psi}) 
\end{aligned}
$$

因此,在轨迹规划时,我们只需要关注四维的平坦输出空间 $(\boldsymbol{p}, \psi)$,而不是十二维的原状态空间 $(\boldsymbol{p}, \boldsymbol{v}, \boldsymbol{a}, \boldsymbol{\Omega}, \dot{\boldsymbol{\Omega}})$,问题复杂度大大降低。

总的来说,微分平坦性通过揭示系统内在的简单结构,允许我们在一个低维空间进行轨迹规划,然后自动满足原系统的动力学约束,提供了一种简化复杂系统控制的强大工具。这就是它在机器人轨迹规划中的重要性。


在本文中,四旋翼无人机系统就具有微分平坦性。选择其位置 $\boldsymbol{p}(t)$ 和偏航角 $\psi(t)$ 作为平坦输出,则速度 $\boldsymbol{v}(t)$、加速度 $\boldsymbol{a}(t)$、姿态角 $\boldsymbol{\Omega}(t)$ 和姿态角速度 $\dot{\boldsymbol{\Omega}}(t)$ 都可以由平坦输出表示:

$$
\begin{aligned}
\boldsymbol{v}(t) &= \dot{\boldsymbol{p}}(t) \
\boldsymbol{a}(t) &= \ddot{\boldsymbol{p}}(t) \
\boldsymbol{\Omega}(t) &= f_1(\boldsymbol{p}, \dot{\boldsymbol{p}}, \ddot{\boldsymbol{p}}, \psi) \
\dot{\boldsymbol{\Omega}}(t) &= f_2(\boldsymbol{p}, \dot{\boldsymbol{p}}, \ddot{\boldsymbol{p}}, \dot{}\ddot{\boldsymbol{p}}, \psi, \dot{\psi})
\end{aligned}
$$

上述公式表示了四旋翼无人机系统的微分平坦性。其中:

- $\boldsymbol{v}(t)$ 是速度,它等于位置 $\boldsymbol{p}(t)$ 的一阶导数。
- $\boldsymbol{a}(t)$ 是加速度,它等于位置 $\boldsymbol{p}(t)$ 的二阶导数。
- $\boldsymbol{\Omega}(t)$ 是姿态角,它可以由位置及其一阶、二阶导数和偏航角 $\psi$ 表示,具体函数关系为 $f_1$。
- $\dot{\boldsymbol{\Omega}}(t)$ 是姿态角速度,它可以由位置及其一至三阶导数、偏航角及其一阶导数表示,具体函数关系为 $f_2$。

对于四旋翼系统,只要规划出位置轨迹 $\boldsymbol{p}(t)$ 和偏航角轨迹 $\psi(t)$,就可以推导出所有其他状态量。这大大简化了轨迹优化问题。


我们只需规划位置轨迹 $\boldsymbol{p}(t)$,其他所有状态量都可由此导出,从而将轨迹优化问题的维度从12降到4,极大地提高了计算效率。同时,只要位置轨迹 $\boldsymbol{p}(t)$ 满足动力学约束(如速度、加速度限制),其他状态量必然也满足相应约束。这保证了生成轨迹的动力学可行性。

综上,微分平坦性通过降维简化了轨迹优化问题,是一个非常有用的系统特性。对于四旋翼等许多机器人系统,利用其微分平坦性可以极大提升轨迹规划的效率和质量,这也是本文采用该特性的原因。

##### 3.附录

这篇paper的附录主要讲述了如何高效地计算 $t^*$ 对优化变量的梯度。这是轨迹优化的一个关键步骤。

在前面的章节中论述了 $t^*$ 是使 $f_{sdf}(\boldsymbol{x}_{ob}, t)$ 最小化的时刻,即满足以下条件:

$$
\dot{f}_{sdf}|_{t^*,\boldsymbol{x}_{ob}} = 0
$$

根据四旋翼的动力学特性,可以将上式简化为:

$$
\dot{f}_{sdf}|_{\boldsymbol{x}_{ob}} = (\nabla SDF_B|_{\boldsymbol{x}_{rel}})^T (\hat{\omega}R^T(\boldsymbol{p} - \boldsymbol{x}_{ob}) - R^T\boldsymbol{v})
$$

其中 $\hat{\omega}$ 是角速度的斜对称矩阵。为方便起见,定义:

$$
\begin{aligned}
X(R, \boldsymbol{p}) &= (\nabla SDF_B|_{\boldsymbol{x}_{rel}})^T \\
Y(R, \hat{\omega}, \boldsymbol{p}, \boldsymbol{v}) &= \hat{\omega}R^T(\boldsymbol{p}-\boldsymbol{x}_{ob})-R^T\boldsymbol{v} \\
F(t^*, \boldsymbol{\zeta}) &= \dot{f}_{sdf}|_{\boldsymbol{x}_{ob}} = X \cdot Y \equiv 0
\end{aligned}
$$

其中 $\boldsymbol{\zeta}$ 表示状态变量 $(\boldsymbol{p}, \boldsymbol{v}, \boldsymbol{\omega}, R)$。

要计算 $\partial t^*/\partial \boldsymbol{\zeta}$,关键是利用隐函数定理。由于 $F(t^*(\boldsymbol{\zeta}), \boldsymbol{\zeta}) \equiv 0$,对其求导可得:

$$
\frac{dF}{d\boldsymbol{\zeta}} = \frac{\partial F}{\partial t^*}\frac{\partial t^*}{\partial \boldsymbol{\zeta}} + \frac{\partial F}{\partial \boldsymbol{\zeta}} \equiv 0
$$

因此:

$$
\frac{\partial t^*}{\partial \boldsymbol{\zeta}} = -\frac{\partial F}{\partial \boldsymbol{\zeta}} / \frac{\partial F}{\partial t^*}
$$

$\partial F/\partial t^*$ 和 $\partial F/\partial \boldsymbol{\zeta}$ 可以通过链式法则计算:

$$
\begin{aligned}
\frac{\partial F}{\partial t^*} &= X\frac{\partial Y}{\partial t^*} + \frac{\partial X}{\partial t^*}Y \\
\frac{\partial F}{\partial \boldsymbol{\zeta}} &= X\frac{\partial Y}{\partial \boldsymbol{\zeta}} + \frac{\partial X}{\partial \boldsymbol{\zeta}}Y
\end{aligned}
$$

其中 $\partial X/\partial t^*, \partial Y/\partial t^*, \partial X/\partial \boldsymbol{\zeta}, \partial Y/\partial \boldsymbol{\zeta}$ 都可以通过机器人和环境的几何关系解析求得（论文中给出了具体公式，这里不再赘述）。

有了 $\partial t^*/\partial \boldsymbol{\zeta}$,再利用链式法则就可以计算 $t^*$ 对优化变量 $\boldsymbol{c}$ 和 $\boldsymbol{T}$ 的梯度:

$$
\frac{\partial t^*}{\partial \boldsymbol{c},\boldsymbol{T}} = \sum_{\boldsymbol{\zeta}=\boldsymbol{p},\boldsymbol{v},\boldsymbol{a},\boldsymbol{j}} \frac{\partial t^*}{\partial \boldsymbol{\zeta}} \cdot \frac{\partial \boldsymbol{\zeta}}{\partial \boldsymbol{c},\boldsymbol{T}}
$$

其中 $\partial \boldsymbol{\zeta}/\partial \boldsymbol{c},\boldsymbol{T}$ 可以通过微分平坦性求得。

上面详细阐述了如何利用隐函数定理和链式法则,高效地计算 $t^*$ 对优化变量的梯度。这避免了对 $t^*$ 的数值微分,大大提高了优化效率。同时,由于所有梯度都有解析表达式,优化过程也更加稳定。

这部分内容通过巧妙地利用隐函数定理,作者成功地将一个看似难以处理的问题（$t^*$ 是通过优化得到的,似乎无法直接求导）转化为了一个可解析求导的问题。这种数学上的创新是本文的一大亮点,也为其高效的轨迹优化算法奠定了基础。


论文可自行搜索英文题目或是去fastlab下载

