from aiohttp import web
import json

routes = web.RouteTableDef()

def create_app(controller):
    app = web.Application()
    app["controller"] = controller

    @routes.get("/status")
    async def status(request):
        mode = controller.mode
        distance = controller.sensor.get_distance()
        return web.json_response({"mode": mode, "distance": distance})

    @routes.post("/mode")
    async def set_mode(request):
        data = await request.json()
        mode = data.get("mode")
        await controller.set_mode(mode)
        return web.json_response({"mode": controller.mode})

    app.add_routes(routes)
    return app

def run_server(controller, host="0.0.0.0", port=8080):
    app = create_app(controller)
    web.run_app(app, host=host, port=port)
