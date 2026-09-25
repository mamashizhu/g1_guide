#!/usr/bin/env bash
# 删除之前放的箱子。用法：bash remove_box.sh <名字>
name=${1:-blocker}
gz service -s /world/exhibition/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 3000 --req "name: \"$name\" type: MODEL"
