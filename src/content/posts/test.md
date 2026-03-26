---
title: 功能测试文章
published: 2026-03-18
description: 测试 Mizuki 博客框架的各种功能
categories:
  - 测试
  - 功能
tags:
  - 测试
  - 功能
author: 测试用户
cover: /assets/images/avatar.webp
draft: false
---

# 功能测试文章

本文用于测试 Mizuki 博客框架的各种功能，包括前言配置、代码高亮、数学公式、表格、图片和视频嵌入。🄋 ➀ ➁ ➂ ➃ ➄ ➅ ➆ ➇ ➈ ➉

## 1. 前言（frontmatter）配置

前言配置已在文件顶部设置，包括标题、日期、描述、分类、标签、作者和封面图片。

## 2. 代码高亮

### JavaScript 代码

```javascript
function helloWorld() {
  console.log('Hello, World!');
  return 'Hello, World!';
}

// 调用函数
helloWorld();
```

### Python 代码

```python
def hello_world():
    print("Hello, World!")
    return "Hello, World!"

# 调用函数
hello_world()
```

### HTML 代码

```html
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>测试页面</title>
</head>
<body>
    <h1>Hello, World!</h1>
    <p>这是一个测试页面</p>
</body>
</html>
```

## 3. 数学公式

### 行内公式

E = mc²

### 块级公式

$$
\int_{a}^{b} f(x) dx = F(b) - F(a)
$$

$$
\sum_{i=1}^{n} i = \frac{n(n+1)}{2}
$$

$$
\lim_{x \to \infty} \frac{1}{x} = 0
$$

## 4. 表格

### 基本表格

| 姓名 | 年龄 | 职业 |
|------|------|------|
| 张三 | 25   | 工程师 |
| 李四 | 30   | 设计师 |
| 王五 | 35   | 教师 |

### 带对齐的表格

| 产品 | 价格 | 库存 |
|------|:----:|------:|
| 商品A | 100  | 10   |
| 商品B | 200  | 5    |
| 商品C | 300  | 8    |

## 5. 图片

### 本地图片

![测试图片](/assets/images/avatar.webp)

### 网络图片

![网络测试图片](https://picsum.photos/800/400)

## 6. 视频嵌入

### YouTube 视频

<iframe width="560" height="315" src="https://www.youtube.com/embed/dQw4w9WgXcQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

### Bilibili 视频

<iframe width="560" height="315" src="https://player.bilibili.com/player.html?aid=123456789&bvid=BV1xx411c7mK&cid=123456789&page=1" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true"></iframe>

## 7. 其他功能测试

### 引用

> 这是一个引用块，用于测试引用功能。
> 可以有多行引用内容。

### 列表

#### 无序列表

- 项目 1
- 项目 2
- 项目 3
  - 子项目 3.1
  - 子项目 3.2

#### 有序列表

1. 第一项
2. 第二项
3. 第三项
   1. 子项 3.1
   2. 子项 3.2

### 强调

**粗体文本**

*斜体文本*

***粗斜体文本***

### 链接

[Mizuki 项目 GitHub](https://github.com/matsuzaka-yuki/Mizuki)

[Mizuki 官方文档](https://mizuki-docs.mysqil.com/)