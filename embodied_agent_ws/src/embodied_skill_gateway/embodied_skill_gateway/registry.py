#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""技能注册表（**纯 Python，不依赖 ROS**）。

注册表是**数据**，不是代码。这是 D-011「新增底层能力时，先在 Skill Manager 注册」
与 D-031「加一个技能不应该需要改接口包重编译」的直接落点。

一个技能在注册表里声明四类东西：

  1. **它是什么** —— 名字、层级（control / primitive / task）、说明
  2. **怎么调它** —— 传输方式与目标（ROS 服务名 + 接口类型），**只有网关看得见**
  3. **允不允许要** —— 参数 schema 与**策略边界**（min / max / 单位），以及谁能调（principal）
  4. **等多久** —— 超时秒数

⚠️ **和 `motion_plan.py` 的分工，别搞混**：

    motion_plan.max_distance   管的是「**能不能做到**」—— 物理/运动学的可行域
    注册表里的 min / max        管的是「**允不允许要**」—— 策略与授权

两层都要，语义不同。注册表里给 `router.deterministic` 的位移上限**刻意小于**
Control Skill 的 `max_distance` —— 网关**必须能拒绝某些底层技能会接受的东西**，
否则它就是个单纯的代理（见 `checks.py` 顶部）。
"""

import json


class RegistryError(Exception):
    """注册表本身有问题。**启动时就要失败得响**，不要等运行到一半才发现技能名写错了。"""


# 参数类型 → 允许的 Python 类型（bool 必须在 int 之前判断：Python 里 bool 是 int 的子类）
_TYPES = {
    'float': (int, float),
    'int': (int,),
    'bool': (bool,),
    'string': (str,),
}

TIERS = ('control', 'primitive', 'task')


class ParamSpec:
    """一个参数的策略描述。"""

    __slots__ = ('name', 'type', 'unit', 'minimum', 'maximum', 'required')

    def __init__(self, name, type='float', unit='', minimum=None, maximum=None,
                 required=True):
        if type not in _TYPES:
            raise RegistryError(f'参数 {name}：未知类型 {type!r}（支持 {sorted(_TYPES)}）')
        self.name = name
        self.type = type
        self.unit = unit
        self.minimum = minimum
        self.maximum = maximum
        self.required = required
        if minimum is not None and maximum is not None and minimum > maximum:
            raise RegistryError(
                f'参数 {name}：下限 {minimum} 大于上限 {maximum}')

    @classmethod
    def from_dict(cls, d):
        if 'name' not in d:
            raise RegistryError(f'参数缺少 name：{d!r}')
        return cls(
            name=d['name'],
            type=d.get('type', 'float'),
            unit=d.get('unit', ''),
            minimum=d.get('min'),
            maximum=d.get('max'),
            required=d.get('required', True),
        )

    def accepts_python_type(self, value):
        """类型对不对。

        ⚠️ `bool` 是 `int` 的子类，所以 `True` 会通过 `int` 的检查 —— 对于
        `float`/`int` 参数这是**错的**（把 `True` 当 1.0 用会掩盖上游的 bug）。
        所以先排 bool，再判类型。
        """
        if isinstance(value, bool):
            return self.type == 'bool'
        return isinstance(value, _TYPES[self.type])

    def __repr__(self):
        return f'ParamSpec({self.name}:{self.type})'


class SkillSpec:
    """一个技能的完整策略描述。"""

    __slots__ = ('name', 'tier', 'description', 'transport', 'target', 'srv_type',
                 'params', 'timeout_s', 'allowed_principals', 'cancel_target',
                 'available', 'unavailable_reason', 'max_norm', 'causes_motion')

    def __init__(self, name, tier, target, srv_type, params=(), timeout_s=10.0,
                 allowed_principals=(), cancel_target=None, transport='service',
                 description='', available=True, unavailable_reason='', max_norm=None,
                 causes_motion=True):
        if tier not in TIERS:
            raise RegistryError(f'技能 {name}：未知层级 {tier!r}（支持 {TIERS}）')
        if not target:
            raise RegistryError(f'技能 {name}：缺少 target（底层服务名）')
        if not srv_type:
            raise RegistryError(f'技能 {name}：缺少 srv_type')
        if not isinstance(timeout_s, (int, float)) or timeout_s <= 0:
            raise RegistryError(f'技能 {name}：timeout_s 必须为正数，得到 {timeout_s!r}')
        if not allowed_principals:
            raise RegistryError(
                f'技能 {name}：allowed_principals 为空 —— 一个谁都调不了的技能是配置错误，'
                f'要停用请用 available: false')
        self.name = name
        self.tier = tier
        self.description = description
        self.transport = transport
        self.target = target
        self.srv_type = srv_type
        self.params = tuple(params)
        self.timeout_s = float(timeout_s)
        self.allowed_principals = frozenset(allowed_principals)
        self.cancel_target = cancel_target
        self.available = bool(available)
        self.unavailable_reason = unavailable_reason
        # ⚠️ **默认 True（保守）**：忘记声明的技能会被当成"会动"，
        #    于是照样受 `allow_motion` 闸门约束。反过来默认 False 的话，
        #    漏写一个字段就等于悄悄开了一道"不受闸门约束"的口子。
        #    只读查询（`primitive.*`）在 YAML 里显式写 `causes_motion: false`。
        self.causes_motion = bool(causes_motion)

        seen = set()
        for p in self.params:
            if p.name in seen:
                raise RegistryError(f'技能 {name}：参数 {p.name} 重复')
            seen.add(p.name)

        # 跨字段约束：|(p1, p2, ...)| ≤ max。
        # 为什么需要它：`move_relative` 的 x 与 y 各自在 ±0.5 内，并不代表
        # **位移**在 0.5 内 —— (0.5, 0.5) 的模长是 0.707。只看单字段的上限
        # 会放行一个比策略允许的更远的请求。
        self.max_norm = None
        if max_norm is not None:
            keys = set(max_norm)
            if keys != {'params', 'max'} and keys != {'params', 'max', 'note'}:
                raise RegistryError(
                    f'技能 {name}：max_norm 必须恰好含 params 与 max（可选 note），得到 {sorted(keys)}')
            names = list(max_norm['params'])
            if len(names) < 1:
                raise RegistryError(f'技能 {name}：max_norm.params 不能为空')
            for n in names:
                if n not in seen:
                    raise RegistryError(
                        f'技能 {name}：max_norm 引用了不存在的参数 {n!r}（已有 {sorted(seen)}）')
            if max_norm['max'] <= 0:
                raise RegistryError(f'技能 {name}：max_norm.max 必须为正')
            self.max_norm = {'params': names, 'max': float(max_norm['max']),
                             'note': max_norm.get('note', '')}

    @classmethod
    def from_dict(cls, name, d):
        try:
            params = [ParamSpec.from_dict(p) for p in d.get('params', [])]
            return cls(
                name=name,
                tier=d['tier'],
                target=d['target'],
                srv_type=d['srv_type'],
                params=params,
                timeout_s=d.get('timeout_s', 10.0),
                allowed_principals=d.get('allowed_principals', []),
                cancel_target=d.get('cancel_target'),
                transport=d.get('transport', 'service'),
                description=d.get('description', ''),
                available=d.get('available', True),
                unavailable_reason=d.get('unavailable_reason', ''),
                max_norm=d.get('max_norm'),
                causes_motion=d.get('causes_motion', True),
            )
        except KeyError as e:
            raise RegistryError(f'技能 {name}：缺少必填字段 {e}') from None

    def param(self, name):
        for p in self.params:
            if p.name == name:
                return p
        return None

    def public_view(self):
        """对外的只读投影（`~/list` 用）。

        ⚠️ **刻意不含 `target`（底层 ROS 服务名）** —— 知道名字不等于能调用，
        调用一律要过 `~/invoke` 的准入。少给一个"看起来像入口"的东西。
        """
        return {
            'tier': self.tier,
            'description': self.description,
            'timeout_s': self.timeout_s,
            'allowed_principals': sorted(self.allowed_principals),
            'available': self.available,
            'unavailable_reason': self.unavailable_reason,
            'params': [
                {'name': p.name, 'type': p.type, 'unit': p.unit,
                 'min': p.minimum, 'max': p.maximum, 'required': p.required}
                for p in self.params
            ],
            'max_norm': self.max_norm,
            'causes_motion': self.causes_motion,
        }

    def __repr__(self):
        return f'SkillSpec({self.name}, tier={self.tier})'


class Registry:
    """技能名 → SkillSpec 的只读表。"""

    def __init__(self, specs):
        self._specs = {}
        for s in specs:
            if s.name in self._specs:
                raise RegistryError(f'技能名重复：{s.name}')
            self._specs[s.name] = s
        if not self._specs:
            raise RegistryError('注册表为空 —— 要么是配置写错了，要么是不该启动网关')

    @classmethod
    def from_dict(cls, data):
        """从已解析的 YAML dict 构造。顶层 key 既可以是 `skills:`，也可以直接是技能名。"""
        if not isinstance(data, dict):
            raise RegistryError(f'注册表顶层必须是 mapping，得到 {type(data).__name__}')
        skills = data.get('skills', data)
        if not isinstance(skills, dict):
            raise RegistryError('skills 必须是 mapping')
        specs = []
        for name, d in skills.items():
            if name == 'skills':
                continue
            if not isinstance(d, dict):
                raise RegistryError(f'技能 {name} 的描述必须是 mapping')
            specs.append(SkillSpec.from_dict(name, d))
        return cls(specs)

    @classmethod
    def from_yaml(cls, path):
        import yaml   # 延迟 import：本模块其余部分与测试不需要 yaml
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        reg = cls.from_dict(data)
        reg.check_against_control_skills_limits()
        return reg

    def unknown_srv_types(self, known):
        """哪些技能的 `srv_type` **不在** `known` 里。返回排好序的列表（空 = 都认识）。

        ⚠️ 为什么要有这一问（2026-10-09 真事）：网关**只调用它认识的服务类型**
        （`skill_gateway._SRV_TYPES` 那张表）。新技能只在注册表里加一条、
        忘了往那张表里登记，**注册表看起来完全正常**：`~/list` 里 `available: true`，
        准入检查也过 —— 一直到**真的有人调它**、且恰好派发到那一步，才报
        `注册表声明了未知的 srv_type …`。

        ⇒ 这种配置错误**必须在启动时就炸**，而不是等某一条任务替它炸
        （与本项目对规则表的态度一致：写错要炸，不要让它表现成"这个技能不好使"）。
        """
        return sorted({s.srv_type for s in self._specs.values()
                       if s.srv_type not in known})

    def get(self, name):
        return self._specs.get(name)

    def require(self, name):
        spec = self._specs.get(name)
        if spec is None:
            raise RegistryError(f'未注册的技能：{name!r}（已注册：{sorted(self._specs)}）')
        return spec

    def names(self):
        return tuple(sorted(self._specs))

    def __len__(self):
        return len(self._specs)

    def __contains__(self, name):
        return name in self._specs

    # ---------- 自校验 ----------

    def check_against_control_skills_limits(self, max_distance=1.0, max_angle=3.141592653589793):
        """**策略边界不得宽于底层技能的可行域**。

        网关的上限比 Control Skill 的 `max_distance` 还大，意味着网关放行的请求
        会一路走到 Control Skill 再被它拒绝 —— 那时拒绝原因来自下层，
        上层的"策略"就名存实亡了。这里在**启动时**把这种配置错误暴露出来。

        （相等是允许的：那表示"策略上就不给更多"，而不是"策略放行了做不到的事"。）
        """
        problems = []
        for name, spec in self._specs.items():
            if spec.tier != 'control':
                continue
            if name.endswith('move_relative'):
                bound = spec.max_norm['max'] if spec.max_norm else None
                if bound is None:
                    problems.append(f'{name} 没有声明 max_norm —— 只看 x/y 的单字段上限'
                                    f'放不住 (0.5, 0.5) 这种组合')
                elif bound > max_distance + 1e-9:
                    problems.append(
                        f'{name} 的位移策略上限 {bound} 超过 Control Skill 的 '
                        f'max_distance {max_distance}')
            if name.endswith('rotate'):
                p = spec.param('angle')
                if p is not None and p.maximum is not None and p.maximum > max_angle + 1e-9:
                    problems.append(
                        f'{name} 的转角策略上限 {p.maximum} 超过 Control Skill 的 '
                        f'max_angle {max_angle}')
        if problems:
            raise RegistryError('注册表与底层技能的能力不一致：\n  - ' + '\n  - '.join(problems))

    def to_json(self):
        """`~/list` 的载荷。"""
        return json.dumps(
            {name: spec.public_view() for name, spec in sorted(self._specs.items())},
            ensure_ascii=False, sort_keys=True)
