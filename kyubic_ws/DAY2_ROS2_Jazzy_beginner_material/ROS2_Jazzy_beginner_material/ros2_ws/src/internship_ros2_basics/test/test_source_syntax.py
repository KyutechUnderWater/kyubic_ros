import ast
from pathlib import Path
import unittest


class SourceSyntaxTest(unittest.TestCase):
    def test_all_python_sources_parse(self):
        package_root = Path(__file__).parents[1]
        paths = list((package_root / "internship_ros2_basics").glob("*.py"))
        paths += list((package_root / "launch").glob("*.py"))
        self.assertGreaterEqual(len(paths), 10)
        for path in paths:
            with self.subTest(path=path.name):
                ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


if __name__ == "__main__":
    unittest.main()
