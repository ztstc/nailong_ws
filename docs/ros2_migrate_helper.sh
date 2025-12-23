#!/bin/bash

# ROS1 到 ROS2 Humble 迁移辅助脚本
# 用途: 自动化一些常见的文件迁移任务
# 使用方法: bash ros2_migrate_helper.sh [命令]

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"

echo -e "${BLUE}ROS1 到 ROS2 Humble 迁移辅助脚本${NC}"
echo "工作空间: $WORKSPACE_ROOT"
echo ""

# 显示帮助
show_help() {
    cat << EOF
使用方法: bash $0 [命令]

命令:
    check-branch          检查当前是否在ros2-humble分支
    check-files          列出所有ROS1相关文件
    backup-launch        备份所有.launch文件
    backup-cmake         备份所有CMakeLists.txt
    backup-package       备份所有package.xml
    check-ros1-headers   检查ROS1头文件使用
    check-nodhandle      检查NodeHandle使用
    check-rosmacros      检查ROS宏使用 (ROS_INFO等)
    clean-build          清理编译目录
    help                 显示此帮助信息

示例:
    bash ros2_migrate_helper.sh check-branch
    bash ros2_migrate_helper.sh check-ros1-headers

EOF
}

# 检查git分支
check_branch() {
    echo -e "${BLUE}检查当前分支...${NC}"
    current_branch=$(cd "$WORKSPACE_ROOT" && git branch --show-current)
    
    if [ "$current_branch" = "ros2-humble" ]; then
        echo -e "${GREEN}✓ 当前在 ros2-humble 分支${NC}"
    else
        echo -e "${YELLOW}⚠ 当前在 $current_branch 分支，建议切换到 ros2-humble${NC}"
        echo "  运行: cd $WORKSPACE_ROOT && git checkout ros2-humble"
    fi
}

# 列出所有ROS1相关文件
check_files() {
    echo -e "${BLUE}扫描ROS1相关文件...${NC}"
    echo ""
    
    echo -e "${YELLOW}CMakeLists.txt 文件:${NC}"
    find "$WORKSPACE_ROOT/src" -name "CMakeLists.txt" 2>/dev/null || echo "  未找到"
    
    echo ""
    echo -e "${YELLOW}package.xml 文件:${NC}"
    find "$WORKSPACE_ROOT/src" -name "package.xml" 2>/dev/null || echo "  未找到"
    
    echo ""
    echo -e "${YELLOW}.launch 文件 (需要转换):${NC}"
    find "$WORKSPACE_ROOT/src" -name "*.launch" 2>/dev/null | head -20 || echo "  未找到"
    
    echo ""
    echo -e "${YELLOW}C++ 源文件:${NC}"
    find "$WORKSPACE_ROOT/src" -name "*.cpp" 2>/dev/null | head -20 || echo "  未找到"
}

# 备份launch文件
backup_launch() {
    echo -e "${BLUE}备份所有.launch文件...${NC}"
    
    backup_dir="$WORKSPACE_ROOT/.ros1_backups/launch"
    mkdir -p "$backup_dir"
    
    count=0
    while IFS= read -r file; do
        cp "$file" "$backup_dir/" 2>/dev/null && ((count++))
    done < <(find "$WORKSPACE_ROOT/src" -name "*.launch" 2>/dev/null)
    
    echo -e "${GREEN}✓ 已备份 $count 个.launch文件${NC}"
    echo "  位置: $backup_dir"
}

# 备份CMakeLists.txt
backup_cmake() {
    echo -e "${BLUE}备份所有CMakeLists.txt...${NC}"
    
    backup_dir="$WORKSPACE_ROOT/.ros1_backups/cmake"
    mkdir -p "$backup_dir"
    
    count=0
    while IFS= read -r file; do
        relative_path=${file#$WORKSPACE_ROOT/}
        mkdir -p "$backup_dir/$(dirname "$relative_path")"
        cp "$file" "$backup_dir/$relative_path" 2>/dev/null && ((count++))
    done < <(find "$WORKSPACE_ROOT/src" -name "CMakeLists.txt" 2>/dev/null)
    
    echo -e "${GREEN}✓ 已备份 $count 个CMakeLists.txt${NC}"
    echo "  位置: $backup_dir"
}

# 备份package.xml
backup_package() {
    echo -e "${BLUE}备份所有package.xml...${NC}"
    
    backup_dir="$WORKSPACE_ROOT/.ros1_backups/package"
    mkdir -p "$backup_dir"
    
    count=0
    while IFS= read -r file; do
        relative_path=${file#$WORKSPACE_ROOT/}
        mkdir -p "$backup_dir/$(dirname "$relative_path")"
        cp "$file" "$backup_dir/$relative_path" 2>/dev/null && ((count++))
    done < <(find "$WORKSPACE_ROOT/src" -name "package.xml" 2>/dev/null)
    
    echo -e "${GREEN}✓ 已备份 $count 个package.xml${NC}"
    echo "  位置: $backup_dir"
}

# 检查ROS1头文件使用
check_ros1_headers() {
    echo -e "${BLUE}检查ROS1头文件使用...${NC}"
    echo ""
    
    echo -e "${YELLOW}包含 <ros/ros.h>:${NC}"
    grep -r "#include <ros/ros.h>" "$WORKSPACE_ROOT/src" --include="*.cpp" --include="*.h" --include="*.hpp" 2>/dev/null | head -10 || echo "  未找到"
    
    echo ""
    echo -e "${YELLOW}包含 <ros/:${NC}"
    grep -r "#include <ros/" "$WORKSPACE_ROOT/src" --include="*.cpp" --include="*.h" --include="*.hpp" 2>/dev/null | wc -l
    echo "  共找到上述数量的ros头文件包含"
}

# 检查NodeHandle使用
check_nodhandle() {
    echo -e "${BLUE}检查ros::NodeHandle使用...${NC}"
    echo ""
    
    count=$(grep -r "ros::NodeHandle" "$WORKSPACE_ROOT/src" --include="*.cpp" --include="*.h" --include="*.hpp" 2>/dev/null | wc -l)
    echo -e "${YELLOW}ros::NodeHandle 出现次数: $count${NC}"
    
    echo ""
    echo "前10个使用位置:"
    grep -r "ros::NodeHandle" "$WORKSPACE_ROOT/src" --include="*.cpp" --include="*.h" --include="*.hpp" 2>/dev/null | head -10
}

# 检查ROS宏使用
check_rosmacros() {
    echo -e "${BLUE}检查ROS日志宏使用...${NC}"
    echo ""
    
    for macro in "ROS_DEBUG" "ROS_INFO" "ROS_WARN" "ROS_ERROR" "ROS_FATAL"; do
        count=$(grep -r "$macro" "$WORKSPACE_ROOT/src" --include="*.cpp" --include="*.h" --include="*.hpp" 2>/dev/null | wc -l)
        if [ "$count" -gt 0 ]; then
            echo -e "${YELLOW}$macro: $count${NC}"
        fi
    done
}

# 清理编译目录
clean_build() {
    echo -e "${BLUE}清理编译目录...${NC}"
    
    for dir in "build" "install" "log"; do
        if [ -d "$WORKSPACE_ROOT/$dir" ]; then
            echo "  删除 $dir/"
            rm -rf "$WORKSPACE_ROOT/$dir"
        fi
    done
    
    echo -e "${GREEN}✓ 清理完成${NC}"
}

# 生成迁移报告
generate_report() {
    echo -e "${BLUE}生成迁移报告...${NC}"
    
    report_file="$WORKSPACE_ROOT/docs/MIGRATION_REPORT.md"
    
    cat > "$report_file" << 'EOF'
# 迁移前分析报告

生成时间: $(date)

## 统计信息

### 文件统计
EOF

    echo "" >> "$report_file"
    echo "#### CMakeLists.txt 文件" >> "$report_file"
    find "$WORKSPACE_ROOT/src" -name "CMakeLists.txt" 2>/dev/null | while read file; do
        echo "- $file" >> "$report_file"
    done
    
    echo "" >> "$report_file"
    echo "#### package.xml 文件" >> "$report_file"
    find "$WORKSPACE_ROOT/src" -name "package.xml" 2>/dev/null | while read file; do
        echo "- $file" >> "$report_file"
    done
    
    echo "" >> "$report_file"
    echo "#### .launch 文件 (需要转换)" >> "$report_file"
    find "$WORKSPACE_ROOT/src" -name "*.launch" 2>/dev/null | while read file; do
        echo "- $file" >> "$report_file"
    done
    
    echo "" >> "$report_file"
    echo "## ROS1 API 使用统计" >> "$report_file"
    echo "" >> "$report_file"
    echo "### ros/ros.h 包含" >> "$report_file"
    count=$(grep -r "#include <ros/ros.h>" "$WORKSPACE_ROOT/src" --include="*.cpp" --include="*.h" --include="*.hpp" 2>/dev/null | wc -l)
    echo "共 $count 处" >> "$report_file"
    
    echo "" >> "$report_file"
    echo "### ros::NodeHandle 使用" >> "$report_file"
    count=$(grep -r "ros::NodeHandle" "$WORKSPACE_ROOT/src" --include="*.cpp" --include="*.h" --include="*.hpp" 2>/dev/null | wc -l)
    echo "共 $count 处" >> "$report_file"
    
    echo "" >> "$report_file"
    echo "### ROS 日志宏使用" >> "$report_file"
    for macro in "ROS_DEBUG" "ROS_INFO" "ROS_WARN" "ROS_ERROR" "ROS_FATAL"; do
        count=$(grep -r "$macro" "$WORKSPACE_ROOT/src" --include="*.cpp" --include="*.h" --include="*.hpp" 2>/dev/null | wc -l)
        if [ "$count" -gt 0 ]; then
            echo "- $macro: $count 处" >> "$report_file"
        fi
    done
    
    echo -e "${GREEN}✓ 报告已生成: $report_file${NC}"
}

# 主程序
main() {
    if [ $# -eq 0 ]; then
        show_help
        exit 0
    fi
    
    case "$1" in
        check-branch)
            check_branch
            ;;
        check-files)
            check_files
            ;;
        backup-launch)
            backup_launch
            ;;
        backup-cmake)
            backup_cmake
            ;;
        backup-package)
            backup_package
            ;;
        check-ros1-headers)
            check_ros1_headers
            ;;
        check-nodhandle)
            check_nodhandle
            ;;
        check-rosmacros)
            check_rosmacros
            ;;
        clean-build)
            clean_build
            ;;
        generate-report)
            generate_report
            ;;
        help)
            show_help
            ;;
        *)
            echo -e "${RED}未知命令: $1${NC}"
            show_help
            exit 1
            ;;
    esac
}

# 运行主程序
main "$@"
