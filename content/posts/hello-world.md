+++
title = '你好，世界：我的第一篇 Hugo 博客'
date = 2026-11-25T10:25:00+08:00
draft = false
tags = ['建站', 'Hugo', '折腾记录']
description = '使用 Hugo 静态博客生成器搭建的第一个测试文章。'
+++

欢迎来到我的第一个个人博客！

## 什么是静态博客？

这篇文章原本只是一个纯文本的 **Markdown** 文件（`content/posts/hello-world.md`）。

当你运行 `hugo` 命令时，Hugo 会把它和 **PaperMod** 主题模板合并，自动生成一套纯静态的 HTML / CSS / JS 网页文件，并输出到 `public/` 目录。

## 后续计划

- [x] 本地安装 Hugo 并初始化站点
- [x] 配置 PaperMod 主题与示例文章
- [x] 推送到 GitHub 并部署到托管平台（Cloudflare Pages / GitHub Pages）
- [ ] 第二阶段：体验在自己的 Vultr VPS 上搭建博客

## 代码高亮测试

```bash
# 本地启动实时预览
hugo server -D

# 生成最终的静态网页文件
hugo --minify
```

写完文章后，只需通过 `git push` 提交到 GitHub，托管平台就会自动重新构建并发布到公网！
