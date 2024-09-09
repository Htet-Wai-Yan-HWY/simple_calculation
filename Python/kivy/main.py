from kivymd.app import MDApp
from kivy.lang import Builder
from kivy.garden.cefpython import CEFBrowser


class WebApp(MDApp):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.screen=Builder.load_file("/home/zyme/kivy_project/web_gui.kv")

    def build(self):
        return self.screen
    


if __name__ == "__main__":

    WebApp().run()