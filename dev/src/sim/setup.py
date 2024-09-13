import os
from setuptools import setup
import xml.etree.ElementTree as ET

package_path = f"{os.path.dirname(os.path.realpath(__file__))}/package.xml"

package = ET.parse(package_path).getroot()
package_name = package.findtext("name")
package_version = package.findtext("version")
package_description = package.findtext("description")
package_license = package.findtext("license")

package_author = package.find("author")
package_maintainer = package.find("maintainer")
package_author_email = package_author.get("email") if package_author is not None else None
package_maintainer_email = package_maintainer.get("email") if package_maintainer is not None else None
package_author = package_author.text if package_author is not None else None
package_maintainer = package_maintainer.text if package_maintainer is not None else None

setup(
    name=package_name,
    version=package_version,
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author=package_author if package_author is not None else package_maintainer,
    author_email=package_author_email if package_author_email is not None else package_maintainer_email,
    maintainer=package_maintainer,
    maintainer_email=package_maintainer_email,
    description=package_description,
    license=package_license,
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
)