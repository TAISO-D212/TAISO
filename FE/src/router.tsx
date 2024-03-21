import { createBrowserRouter } from 'react-router-dom';
import { LoginPage } from './pages/login/LoginPage.tsx';
import { MainPage } from './pages/main/MainPage.tsx';
import { NotFound } from './pages/NotFound.tsx';
import { UserPage } from './pages/profile/UserPage.tsx';

export const router = createBrowserRouter([
  {path: '/', element: <LoginPage />},
  {path: '/main', element: <MainPage />},
  {path: '/profile', element: <UserPage />},
  {path: '/*', element: <NotFound />}
]);
