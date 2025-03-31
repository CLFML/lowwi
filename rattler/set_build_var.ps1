# Get version from latest GitHub release (strip 'v')
$env:package_ver = (gh release view --json tagName | ConvertFrom-Json).tagName -replace '^v', ''

# Get GitHub repo URL
$env:package_gh = (gh repo view --json url | ConvertFrom-Json).url

# Convert GitHub repo URL to GitHub Pages URL
$repo_info = gh repo view --json owner,name | ConvertFrom-Json
$repo_owner = $repo_info.owner.login.ToLower()
$repo_name = $repo_info.name
$env:package_documentation_link = "https://${repo_owner}.github.io/${repo_name}"

# Parse package.xml if available
if (Test-Path "package.xml") {
    try {
        [xml]$xml = Get-Content package.xml
        $ros_package_name = $xml.package.name.Trim()
        $env:package_name = $ros_package_name.ToLower()

        $env:package_desc = $xml.package.description.Trim()
        $env:package_maintainer_email = $xml.package.maintainer.email

        $maintainer_raw = $xml.package.maintainer.InnerXml
        $env:package_maintainer = $xml.package.maintainer.InnerText -replace '\s*<[^>]+>', ''
    } catch {
        $env:package_name = "unknown"
        $env:package_desc = "Custom ROS2 package template"
        $env:package_maintainer = "Unknown"
        $env:package_maintainer_email = "unknown@example.com"
    }
} else {
    $env:package_name = "unknown"
    $env:package_desc = "Custom ROS2 package template"
    $env:package_maintainer = "Unknown"
    $env:package_maintainer_email = "unknown@example.com"
}

# Fallback version if release is not tagged
if (-not $env:package_ver) {
    $env:package_ver = "1.0.0"
}

# Ensure additional expected vars for rattler
$env:package_summary = $env:package_desc
$env:repository = $env:package_gh

# Print for visibility
Write-Host "package_name           = $env:package_name"
Write-Host "version                = $env:package_ver"
Write-Host "repo                   = $env:package_gh"
Write-Host "description            = $env:package_desc"
Write-Host "maintainer             = $env:package_maintainer"
Write-Host "documentation_link     = $env:package_documentation_link"
