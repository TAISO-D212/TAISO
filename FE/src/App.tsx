import { MainLayout } from "./layouts/MainLayout";
import { RouterProvider } from "react-router-dom";
import { router } from "./router";

export const App = () => {

  return (
    <>
      <MainLayout>
        <RouterProvider router={router}/>
      </MainLayout>
    </>
  );
};
