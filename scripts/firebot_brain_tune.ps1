# Tune brain_node from Windows (Docker Desktop). Same ROS params as firebot_brain_tune.sh.
#
# Examples:
#   .\scripts\firebot_brain_tune.ps1 help
#   .\scripts\firebot_brain_tune.ps1 status
#   $env:FIREBOT_USE_DOCKER = "1"; .\scripts\firebot_brain_tune.ps1 mode pulse
#
param(
    [Parameter(Position = 0)][string]$Command = "help",
    [Parameter(Position = 1, ValueFromRemainingArguments = $true)][string[]]$ArgsRemain = @()
)

$ErrorActionPreference = "Stop"
$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

$Node = if ($env:FIREBOT_BRAIN_NODE) { $env:FIREBOT_BRAIN_NODE } else { "/brain_node" }
$DockerDir = if ($env:FIREBOT_DOCKER_DIR) { $env:FIREBOT_DOCKER_DIR } else { Join-Path $Root "docker" }
$ComposeFiles = if ($env:FIREBOT_COMPOSE_FILES) { $env:FIREBOT_COMPOSE_FILES } else { "-f docker-compose.yml -f compose.host-vision-udp-simple.yml" }
$Service = if ($env:FIREBOT_DOCKER_SERVICE) { $env:FIREBOT_DOCKER_SERVICE } else { "firebot" }

function Invoke-FirebotRos2 {
    param([string[]]$RosArgs)
    if ($env:FIREBOT_USE_DOCKER -eq "1") {
        Push-Location $DockerDir
        try {
            $cmd = "docker compose $ComposeFiles exec -T $Service ros2 $($RosArgs -join ' ')"
            Invoke-Expression $cmd
        } finally {
            Pop-Location
        }
    } else {
        & ros2 @RosArgs
    }
}

function Set-BrainParam {
    param([string]$Name, [string]$Value)
    Invoke-FirebotRos2 @("param", "set", $Node, $Name, $Value)
}

switch ($Command) {
    "help" {
        Get-Content $PSCommandPath | Select-String "^#" | ForEach-Object { $_.Line.TrimStart("# ") }
        Write-Host @"

Commands: help, status, start-docker, start-hostvision, mode, rotate, rest, interval, speed, v_approach, kp_yaw, corner_vx, alternate, raw

  .\scripts\firebot_brain_tune.ps1 mode pulse
  .\scripts\firebot_brain_tune.ps1 rotate 0.35
  `$env:FIREBOT_USE_DOCKER='1'; .\scripts\firebot_brain_tune.ps1 status

Raw: .\scripts\firebot_brain_tune.ps1 raw simple_pulse_seek_sign 1
"@ 
    }
    "status" {
        foreach ($n in @(
            "simple_seek_mode", "simple_pulse_rotate_sec", "simple_pulse_rest_sec",
            "simple_pulse_min_interval_sec", "rotate_speed", "v_approach", "kp_yaw",
            "corner_exit_speed", "simple_pulse_seek_alternate"
        )) {
            Write-Host "--- $n"
            try { Invoke-FirebotRos2 @("param", "get", $Node, $n) } catch { Write-Host $_ }
        }
    }
    "start-docker" {
        Write-Host "cd `"$DockerDir`"; docker compose -f docker-compose.yml -f compose.host-vision-udp-simple.yml up -d"
    }
    "start-hostvision" {
        $p = if ($env:FIREBOT_UDP_DETECTION_PORT) { $env:FIREBOT_UDP_DETECTION_PORT } else { "7766" }
        Write-Host "python3 `"$Root\scripts\rpi_test_yolo_fire.py`" --video-mode --udp-bridge 127.0.0.1:$p"
    }
    "mode" {
        $m = $ArgsRemain[0]
        if ($m -notmatch "^(continuous|pulse)$") { throw "usage: mode continuous|pulse" }
        Set-BrainParam "simple_seek_mode" $m
    }
    "rotate" { Set-BrainParam "simple_pulse_rotate_sec" $ArgsRemain[0] }
    "rest" { Set-BrainParam "simple_pulse_rest_sec" $ArgsRemain[0] }
    "interval" { Set-BrainParam "simple_pulse_min_interval_sec" $ArgsRemain[0] }
    "speed"             { Set-BrainParam "rotate_speed" $ArgsRemain[0] }
    "v_approach"        { Set-BrainParam "v_approach" $ArgsRemain[0] }
    "kp_yaw"            { Set-BrainParam "kp_yaw" $ArgsRemain[0] }
    "corner_vx"         { Set-BrainParam "corner_exit_speed" $ArgsRemain[0] }
    "alternate"         { Set-BrainParam "simple_pulse_seek_alternate" $ArgsRemain[0] }
    "raw" {
        if ($ArgsRemain.Count -lt 2) { throw "usage: raw <param_name> <value>" }
        Set-BrainParam $ArgsRemain[0] $ArgsRemain[1]
    }
    default { throw "unknown command: $Command (try help)" }
}
