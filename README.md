# Vision Knowledge Base - 中维动力视觉知识库

基于 [Docusaurus 2](https://docusaurus.io/) 搭建的知识库  

在线访问地址：https://fjut-vision-blog.vercel.app/


# 使用介绍

## 如何上传本地文件至 Vision Knowledge Base
#### 1.使用git将项目克隆到本地
> 首先更新 fork，然后再进行拉取
```
git clone https://github.com/typical-motion/fjut-vision-blog.git
```

#### 2.在``docs``目录下编写``.md``文件

**Tips:** 如果需要图片，则需要将图片添加到``static``目录下  
在``.md``文件下使用相对路径

#### 3.在本地验证是否有bug  
```yarn build```  
（对大小写敏感、本地能过服务器build不一定能过）

#### 4.上传代码至GitHub  

```
git add . 
git commit -m "简介"  
git push
```
如果提交了错误的可以从`base`进行更新，为了预防出现意外可以新建`branch`然后`pull`确认没有问题之后进行`merge`

#### 5.pull to base 

确认没有问题之后记得 `pull requests`
