#!/bin/bash
# -*- coding: utf-8 -*-
#
# 语音合成脚本 - 生成4个场馆的 back_xxx.pcm 文件
#
# 使用方法：
#   确保比赛服务已启动 → 直接运行：
#   bash src/reicoures_facedetect/scripts/synthesize_back_audio.sh
#
# TTS服务返回相对路径 ./REIAIUI/audio/tts.pcm
# 脚本自动在常见路径中查找该文件

ROS_WS="/home/reicom2025/ros_workspace"
BOBAC3_WS="/home/reicom2025/bobac3_ws"
AUDIO_DIR="$BOBAC3_WS/src/robot_audio/audio"

source "$ROS_WS/devel/setup.bash"

echo "============================================"
echo "  语音合成 - 生成4个场馆的back音频"
echo "============================================"

# 检查TTS服务
if ! rosservice list 2>/dev/null | grep -q "/REIService/voice_tts"; then
    echo "❌ TTS服务未运行！请先执行: bash start_competition.sh"
    exit 1
fi

# 逐个场馆生成
for entry in "北京馆:beijing" "上海馆:shanghai" "广州馆:guangzhou" "吉林馆:jilin"; do
    venue=$(echo $entry | cut -d: -f1)
    pinyin=$(echo $entry | cut -d: -f2)
    text="这里就是${venue}啦，我要继续回去工作啦！"
    filename="back_${pinyin}.pcm"
    filepath="${AUDIO_DIR}/${filename}"

    echo ""
    echo "--- 合成 $venue ($filename) ---"
    echo "文本: \"$text\""

    # 调用TTS（用is_play=true确保文件生成并播放）
    rosservice call /REIService/voice_tts "{text: '$text', is_play: true}" > /tmp/tts_result.txt 2>&1

    if grep -q "success: True" /tmp/tts_result.txt; then
        # TTS返回相对路径 ./REIAIUI/audio/tts.pcm
        # 等待2秒确保文件写入完成
        sleep 2

        # 在多个可能路径中查找tts.pcm
        found=""
        for try_path in \
            "$BOBAC3_WS/src/rei_voice/REIAIUI/audio/tts.pcm" \
            "$ROS_WS/src/rei_voice/REIAIUI/audio/tts.pcm" \
            "/home/reicom2025/REIAIUI/audio/tts.pcm" \
            "/tmp/tts.pcm" \
            "$(find /home/reicom2025 -name "tts.pcm" -newer /tmp/tts_result.txt 2>/dev/null | head -1)"; do
            if [ -f "$try_path" ]; then
                found="$try_path"
                break
            fi
        done

        if [ -n "$found" ]; then
            cp "$found" "$filepath"
            echo "✅ $filename 合成成功！保存到: $filepath ($(du -h "$filepath" | cut -f1))"
        else
            echo "⚠️  找不到tts.pcm临时文件，尝试全局搜索..."
            # 最后尝试用find搜索
            found_file=$(find /home/reicom2025 -name "tts.pcm" -mmin -1 2>/dev/null | head -1)
            if [ -n "$found_file" ]; then
                cp "$found_file" "$filepath"
                echo "✅ 已找到并复制: $found_file -> $filepath"
            else
                echo "❌ 实在找不到tts.pcm文件，请手动查找后复制"
                echo "   命令建议: find /home/reicom2025 -name \"tts.pcm\" -mmin -1"
            fi
        fi
    else
        echo "❌ TTS调用失败"
        cat /tmp/tts_result.txt
    fi
    sleep 2
done

echo ""
echo "============================================"
echo "✅ 完成！检查生成的文件:"
echo "============================================"
ls -la "$AUDIO_DIR"/back_*.pcm 2>/dev/null || echo "(无文件生成)"
