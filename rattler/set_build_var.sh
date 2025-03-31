# Get version from latest GitHub release (strip 'v')
export package_ver="$(gh release view --json tagName --jq .tagName | sed 's/^v//')"

# Get GitHub repo URL
export package_gh="$(gh repo view --json url --jq .url)"

# Convert GitHub repo URL to GitHub Pages URL
repo_owner="$(gh repo view --json owner --jq .owner.login | tr '[:upper:]' '[:lower:]')"
repo_name="$(gh repo view --json name --jq .name)"
export package_documentation_link="https://${repo_owner}.github.io/${repo_name}"

# Parse package.xml if available
if [ -f package.xml ]; then
  ros_package_name="$(xmllint --xpath 'normalize-space(/package/name)' package.xml 2>/dev/null || echo 'unknown')"
  export package_name="$(echo "$ros_package_name" | tr '[:upper:]' '[:lower:]')"

  export package_desc="$(xmllint --xpath 'normalize-space(/package/description)' package.xml 2>/dev/null || echo 'Custom ROS2 package template')"
  export package_maintainer_email="$(xmllint --xpath 'string(/package/maintainer/@email)' package.xml 2>/dev/null || echo 'unknown@example.com')"

  maintainer_raw="$(xmllint --xpath 'normalize-space(/package/maintainer)' package.xml 2>/dev/null || echo 'Unknown')"
  export package_maintainer="$(echo "$maintainer_raw" | sed -E 's/ *<[^>]+>//g')"
else
  export package_name="unknown"
  export package_desc="Custom ROS2 package template"
  export package_maintainer="Unknown"
  export package_maintainer_email="unknown@example.com"
fi

# Fallback version if release is not tagged
if [ -z "$package_ver" ]; then
  export package_ver="1.0.0"
fi

# Ensure expected aliases
export package_summary="$package_desc"
export repository="$package_gh"

# Print for visibility
echo "package_name           = $package_name"
echo "version                = $package_ver"
echo "repo                   = $package_gh"
echo "description            = $package_desc"
echo "summary                = $package_summary"
echo "maintainer             = $package_maintainer"
echo "documentation_link     = $package_documentation_link"
