
import kivy                                                                                     
from kivy.app import App                                                                        
from kivy.lang import Builder                                                                   
from kivy.utils import platform                                                                 
from kivy.uix.widget import Widget                                                              
from kivy.clock import Clock                                                                    
                                                                    
                                                                                
class Wv(Widget):                                                                               
    def __init__(self, **kwargs):                                                               
        super(Wv, self).__init__(**kwargs)                                                      
        Clock.schedule_once(self.create_webview, 0)                                             
                                                                                                
                                                                         
    def create_webview(self, *args):                                                            
        webview = WebView(activity)                                                             
        webview.getSettings().setJavaScriptEnabled(True)                                        
        wvc = WebViewClient();                                                                  
        webview.setWebViewClient(wvc);                                                          
        activity.setContentView(webview)                                                        
        webview.loadUrl('http://www.google.com')
                                                                                                
class ServiceApp(App):                                                                          
    def build(self):                                                                            
        return Wv()                                                                             
                                                                                                
if __name__ == '__main__':                                                                      
    ServiceApp().run()