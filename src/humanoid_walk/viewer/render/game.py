"""The main entry point for the humanoid walk viewer application."""

from humanoid_walk.config import load_config
from humanoid_walk.viewer.render.app import App


def main():
    """Run the humanoid walk viewer application."""
    # Create and run the application
    App(load_config()).run()


if __name__ == "__main__":
    main()
