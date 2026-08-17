# ============================================================
# xr_teleoperate 部署脚本 (PowerShell)
# 功能: Git 推送 + Docker 构建 + Docker Hub 推送
# 使用方法: .\scripts\deploy.ps1
# ============================================================
param(
    [switch]$SkipGit,
    [switch]$SkipDocker,
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

# 配置
$GitHubRepo = "illuvorite/xr_teleoperate"
$DockerHubUser = "dopamineillusory"
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$DeployDir = Join-Path $ProjectRoot "deploy"

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host " xr_teleoperate 部署脚本" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host ""

# 检查 Git
if (-not $SkipGit) {
    Write-Host "[1/4] 推送代码到 GitHub..." -ForegroundColor Yellow
    
    if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
        Write-Host "ERROR: git 未安装" -ForegroundColor Red
        exit 1
    }
    
    Push-Location $ProjectRoot
    
    # 检查远程仓库
    $origin = git remote get-url origin 2>$null
    if (-not $origin) {
        Write-Host "添加远程仓库: https://github.com/$GitHubRepo.git"
        git remote add origin "https://github.com/$GitHubRepo.git"
    } elseif ($origin -ne "https://github.com/$GitHubRepo.git") {
        Write-Host "更新远程仓库地址: https://github.com/$GitHubRepo.git"
        git remote set-url origin "https://github.com/$GitHubRepo.git"
    }
    
    # 初始化子模块
    Write-Host "初始化 Git 子模块..."
    git submodule update --init --depth 1
    
    # 推送主仓库
    Write-Host "推送主仓库到 GitHub..."
    git push -u origin master
    
    # 推送子模块
    Write-Host "推送子模块到 GitHub..."
    git submodule foreach {
        $current = $_
        if ($current -match "Entering '(.+)'") {
            $submodulePath = $Matches[1]
        }
        Push-Location $submodulePath
        try {
            git push -u origin HEAD:master
        } catch {
            Write-Host "  警告: 子模块 $submodulePath 推送失败: $_" -ForegroundColor Yellow
        }
        Pop-Location
    }
    
    Pop-Location
    Write-Host "✓ GitHub 推送完成" -ForegroundColor Green
} else {
    Write-Host "[1/4] 跳过 Git 推送" -ForegroundColor Gray
}

# 检查 Docker
if (-not $SkipDocker) {
    Write-Host ""
    Write-Host "[2/4] 构建 Docker 镜像..." -ForegroundColor Yellow
    
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
        Write-Host "ERROR: docker 未安装" -ForegroundColor Red
        exit 1
    }
    
    if (-not (docker info >/dev/null 2>&1)) {
        Write-Host "ERROR: Docker 未运行，请先启动 Docker Desktop" -ForegroundColor Red
        exit 1
    }
    
    Push-Location $DeployDir
    
    # 检查 .env 文件
    if (-not (Test-Path ".env")) {
        Write-Host "创建 .env 文件..."
        Copy-Item ".env.example" ".env"
        Write-Host "请编辑 .env 文件，设置 IMG_SERVER_IP 等配置" -ForegroundColor Yellow
    }
    
    # 构建镜像
    Write-Host "构建镜像 (这可能需要几分钟)..."
    docker compose build
    
    Write-Host "✓ Docker 镜像构建完成" -ForegroundColor Green
    
    # 标记镜像
    Write-Host ""
    Write-Host "[3/4] 标记镜像..." -ForegroundColor Yellow
    
    $xrImage = "$DockerHubUser/xr-teleoperate:latest"
    $teleImage = "$DockerHubUser/teleimager:latest"
    
    docker tag xr-teleoperate:latest $xrImage
    docker tag teleimager:latest $teleImage
    
    Write-Host "标记完成:"
    Write-Host "  $xrImage"
    Write-Host "  $teleImage"
    
    # 推送镜像
    Write-Host ""
    Write-Host "[4/4] 推送到 Docker Hub..." -ForegroundColor Yellow
    
    # 检查登录状态
    $dockerInfo = docker info
    if ($dockerInfo -notmatch "Username: $DockerHubUser") {
        Write-Host "请先登录 Docker Hub: docker login" -ForegroundColor Yellow
        $login = Read-Host "是否现在登录? (y/N)"
        if ($login -eq "y" -or $login -eq "Y") {
            docker login
        } else {
            Write-Host "跳过推送，请稍后手动执行: docker push $xrImage" -ForegroundColor Yellow
            Pop-Location
            return
        }
    }
    
    docker push $xrImage
    docker push $teleImage
    
    Write-Host "✓ Docker Hub 推送完成" -ForegroundColor Green
    
    Pop-Location
} else {
    Write-Host "[2/4] 跳过 Docker 构建" -ForegroundColor Gray
    Write-Host "[3/4] 跳过 Docker 标记" -ForegroundColor Gray
    Write-Host "[4/4] 跳过 Docker 推送" -ForegroundColor Gray
}

Write-Host ""
Write-Host "==========================================" -ForegroundColor Green
Write-Host " 部署完成！" -ForegroundColor Green
Write-Host "==========================================" -ForegroundColor Green
Write-Host ""
Write-Host "其他机器部署命令:" -ForegroundColor Cyan
Write-Host "  git clone --depth 1 https://github.com/$GitHubRepo.git"
Write-Host "  cd xr_teleoperate/deploy"
Write-Host "  cp .env.example .env"
Write-Host "  docker compose -f docker-compose.remote.yml pull"
Write-Host "  docker compose -f docker-compose.remote.yml up -d"
