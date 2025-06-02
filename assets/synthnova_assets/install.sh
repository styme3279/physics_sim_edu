# Herman Ye@Galbot 2025-05-09

# 检测是否启用quiet模式
QUIET_MODE=false
if [ "$1" = "quiet" ]; then
    QUIET_MODE=true
fi

# 检测当前shell类型
detect_shell() {
    # 获取父进程的shell
    parent_shell=$(ps -p $PPID -o comm= 2>/dev/null)

    # 根据父进程shell判断类型
    case "$parent_shell" in
    *bash*)
        echo "bash"
        ;;
    *zsh*)
        echo "zsh"
        ;;
    *)
        # 如果无法确定，尝试通过环境变量判断
        if [ -n "$BASH_VERSION" ]; then
            echo "bash"
        elif [ -n "$ZSH_VERSION" ]; then
            echo "zsh"
        else
            echo "sh"
        fi
        ;;
    esac
}

# 获取shell配置文件路径
get_shell_rc() {
    local shell_type=$(detect_shell)
    case "$shell_type" in
    "bash")
        echo "$HOME/.bashrc"
        ;;
    "zsh")
        echo "$HOME/.zshrc"
        ;;
    *)
        echo "$HOME/.profile"
        ;;
    esac
}

# 检查环境变量是否存在
check_env_variable() {
    if [ -n "$SYNTHNOVA_ASSETS" ]; then
        echo "当前Synthnova默认资产目录: $SYNTHNOVA_ASSETS"
        return 0
    else
        return 1
    fi
}

# 检查git是否安装
check_git() {
    if ! command -v git >/dev/null 2>&1; then
        echo "错误: 未找到git命令，请先安装git"
        exit 1
    fi
}

# 创建资产目录
create_assets_directory() {
    default_path="$HOME/sn_assets"
    shell_rc=$(get_shell_rc)

    if [ "$QUIET_MODE" = true ]; then
        assets_path="$default_path"
    else
        while true; do
            read -p "请输入你希望的资产文件夹路径 (直接回车使用默认路径 $default_path): " user_input

            if [ -z "$user_input" ]; then
                assets_path="$default_path"
            else
                assets_path="$user_input"
            fi
            break
        done
    fi

    # 创建目录
    if mkdir -p "$assets_path" 2>/dev/null; then
        echo "已创建目录: $(realpath "$assets_path")"

        # 设置环境变量
        export SYNTHNOVA_ASSETS="$assets_path"
        echo "已设置环境变量 SYNTHNOVA_ASSETS"

        # 将环境变量添加到对应的shell配置文件
        if ! grep -q "export SYNTHNOVA_ASSETS=" "$shell_rc"; then
            echo "export SYNTHNOVA_ASSETS=\"$assets_path\"" >>"$shell_rc"
            echo "已将环境变量添加到 $(basename "$shell_rc")"
        fi

        # 如果使用zsh，也添加到.zshenv
        if [ "$(detect_shell)" = "zsh" ] && [ -f "$HOME/.zshenv" ]; then
            if ! grep -q "export SYNTHNOVA_ASSETS=" "$HOME/.zshenv"; then
                echo "export SYNTHNOVA_ASSETS=\"$assets_path\"" >>"$HOME/.zshenv"
                echo "已将环境变量添加到 .zshenv"
            fi
        fi

        # 提醒用户重新打开终端或source配置文件
        echo "注意： 在更新环境变量后请重新打开终端或source $shell_rc 以使环境变量生效"
        return 0
    else
        echo "创建目录时出错"
        if [ "$QUIET_MODE" = true ]; then
            exit 1
        fi
        read -p "是否重试? (y/n): " retry
        if [ "$retry" != "y" ]; then
            exit 1
        fi
    fi
}

# 更新资产仓库
update_assets_repo() {
    local assets_dir="$SYNTHNOVA_ASSETS/synthnova_assets"
    local repo_url="ssh://git@git.galbot.com:6043/synth_nova/basic/synthnova_assets.git"
    local branch="develop"

    # 检查目录是否存在
    if [ -d "$assets_dir" ]; then
        # 检查是否是git仓库
        if [ -d "$assets_dir/.git" ]; then
            echo "检测到synthnova_assets仓库，正在检查状态..."

            # 进入目录
            cd "$assets_dir" || {
                echo "错误: 无法进入目录 $assets_dir"
                exit 1
            }

            # 获取远程信息
            if ! git remote show origin >/dev/null 2>&1; then
                echo "警告: 远程仓库信息不完整，将重新克隆"
                cd - >/dev/null || exit 1
                rm -rf "$assets_dir"
            else
                # 获取远程分支信息
                if ! git fetch origin; then
                    echo "错误: 无法获取远程仓库信息"
                    cd - >/dev/null || exit 1
                    rm -rf "$assets_dir"
                else
                    # 检查是否有任何差异（包括未提交的更改、未推送的提交或远程更新）
                    if ! git diff --quiet HEAD || ! git diff --quiet origin/$branch..HEAD; then
                        echo "发现本地与远程仓库存在差异，将备份并重新克隆"
                        cd - >/dev/null || exit 1
                        mv "$assets_dir" "${assets_dir}_backup_$(date +%Y%m%d_%H%M%S)"
                    else
                        echo "synthnova_assets已是最新版本"
                        cd - >/dev/null || exit 1
                        return 0
                    fi
                fi
            fi
        else
            echo "发现synthnova_assets目录但不是git仓库，将备份并重新克隆"
            mv "$assets_dir" "${assets_dir}_backup_$(date +%Y%m%d_%H%M%S)"
        fi
    fi

    # 确保父目录存在
    mkdir -p "$(dirname "$assets_dir")" || {
        echo "错误: 无法创建目录 $(dirname "$assets_dir")"
        exit 1
    }

    # 克隆仓库（只拉取最新版本）
    echo "正在克隆synthnova_assets仓库（仅最新版本）..."
    if ! git clone --depth 1 -b "$branch" "$repo_url" "$assets_dir"; then
        echo "错误: 克隆仓库失败，请检查网络连接和SSH密钥配置"
        exit 1
    fi

    echo "synthnova_assets更新完成！"
}

# 主函数
main() {
    echo "欢迎使用 SynthNova 默认资产更新工具"
    echo "当前Shell: $(detect_shell)"

    # 检查环境变量
    if ! check_env_variable; then
        echo "警告： 未找到环境变量 SYNTHNOVA_ASSETS"
        create_assets_directory
    fi

    # 检查git是否安装
    check_git

    # 询问是否需要更新默认资产
    if [ "$QUIET_MODE" = true ]; then
        update_latest_assets="y"
    else
        echo
        read -p "是否需要更新最新的Synthnova默认资产? (y/n) [y]: " update_latest_assets
        update_latest_assets=${update_latest_assets:-y}
    fi

    if [ "$update_latest_assets" != "y" ]; then
        echo "已跳过更新"
        exit 0
    else
        echo
        echo "开始更新Synthnova默认资产..."
        # 更新资产仓库
        update_assets_repo
    fi
}

# 运行主函数
main
