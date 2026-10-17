# Creates the two directory junctions the Unreal setup needs. Safe to run repeatedly.
#   UnrealHost/Plugins/SwarmSim                                  -> ue5-plugin/SwarmSim
#   ue5-plugin/SwarmSim/Source/SwarmSim/Private/swarm_format -> core/format
# Junctions do not need administrator rights. Both paths are in .gitignore.

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot

function New-RepoJunction($link, $target) {
    if (Test-Path $link) {
        Write-Host "exists: $link"
        return
    }
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $link) | Out-Null
    New-Item -ItemType Junction -Path $link -Target $target | Out-Null
    Write-Host "linked: $link -> $target"
}

New-RepoJunction (Join-Path $root 'UnrealHost\Plugins\SwarmSim') (Join-Path $root 'ue5-plugin\SwarmSim')
New-RepoJunction (Join-Path $root 'ue5-plugin\SwarmSim\Source\SwarmSim\Private\swarm_format') (Join-Path $root 'core\format')
